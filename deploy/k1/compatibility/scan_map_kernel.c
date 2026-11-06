/* Optional K1 acceleration; retain NumPy's distance and reduction semantics. */
#include <math.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>

int rvclaw_map_kernel_abi(void) { return 1; }

int rvclaw_chamfer(const uint8_t *occupied, float *out, int h, int w) {
    if (!occupied || !out || h <= 0 || w <= 0) return -1;
    const double diagonal = sqrt(2.0);
    for (size_t i = 0; i < (size_t)h*w; ++i)
        out[i] = occupied[i] ? 0.0f : INFINITY;
    for (int y = 0; y < h; ++y) for (int x = 0; x < w; ++x) {
        size_t i = (size_t)y*w+x;
        double v = out[i];
        if (x) v = fmin(v, (double)out[i-1]+1.0);
        if (y) {
            v = fmin(v, (double)out[i-w]+1.0);
            if (x) v = fmin(v, (double)out[i-w-1]+diagonal);
            if (x+1<w) v = fmin(v, (double)out[i-w+1]+diagonal);
        }
        out[i] = (float)v;
    }
    for (int y = h-1; y >= 0; --y) for (int x = w-1; x >= 0; --x) {
        size_t i = (size_t)y*w+x;
        double v = out[i];
        if (x+1<w) v = fmin(v, (double)out[i+1]+1.0);
        if (y+1<h) {
            v = fmin(v, (double)out[i+w]+1.0);
            if (x) v = fmin(v, (double)out[i+w-1]+diagonal);
            if (x+1<w) v = fmin(v, (double)out[i+w+1]+diagonal);
        }
        out[i] = (float)v;
    }
    return 0;
}

/* NumPy uses 8 accumulators and recursively bisects blocks above 128. */
static float pairwise_sum(const float *a, size_t n) {
    if (n < 8) {
        float result = -0.0f;
        for (size_t i = 0; i < n; ++i) result += a[i];
        return result;
    }
    if (n <= 128) {
        float r[8];
        for (int k = 0; k < 8; ++k) r[k] = a[k];
        size_t i = 8;
        for (; i < n-(n%8); i+=8)
            for (int k = 0; k < 8; ++k) r[k] += a[i+k];
        float result = ((r[0]+r[1])+(r[2]+r[3]))+((r[4]+r[5])+(r[6]+r[7]));
        for (; i < n; ++i) result += a[i];
        return result;
    }
    size_t half = n/2;
    half -= half%8;
    return pairwise_sum(a, half)+pairwise_sum(a+half, n-half);
}

/* Unrotated maps repeat the same X/Y coordinates across a search grid.
 * Reuse exact cell divisions for identical coordinates, including boundary
 * checks. Do not replace division with reciprocal multiplication or alter
 * endpoint/reduction order. Storage is per call for concurrent scan searches.
 * Return zero on allocation/budget failure so the original loop can run. */
static int score_axis_cached(const int8_t *cells, const float *field, int h, int w,
                             double resolution, double ox, double oy,
                             const double *positions, size_t count,
                             const double *rx, const double *ry, size_t points,
                             float *work, double *scores, double *known_ratios) {
    if (!count || count > 65536 || points > 4096 ||
        count > ((size_t)16*1024*1024)/(2*sizeof(int)*points)) return 0;
    int *xc = malloc(count*points*sizeof(int));
    int *yc = malloc(count*points*sizeof(int));
    size_t *xi = malloc(count*sizeof(size_t));
    size_t *yi = malloc(count*sizeof(size_t));
    double *xs = malloc(count*sizeof(double));
    double *ys = malloc(count*sizeof(double));
    if (!xc || !yc || !xi || !yi || !xs || !ys) {
        free(xc); free(yc); free(xi); free(yi); free(xs); free(ys);
        return 0;
    }
    size_t nx=0, ny=0;
    for (size_t p=0; p<count; ++p) {
        size_t ix=0, iy=0;
        while (ix<nx && xs[ix]!=positions[2*p]) ++ix;
        while (iy<ny && ys[iy]!=positions[2*p+1]) ++iy;
        if (ix==nx) {
            xs[nx++]=positions[2*p];
            for (size_t k=0; k<points; ++k) {
                double cx=((positions[2*p]+rx[k])-ox)/resolution;
                xc[ix*points+k]=(cx>=0 && cx<w) ? (int)cx : -1;
            }
        }
        if (iy==ny) {
            ys[ny++]=positions[2*p+1];
            for (size_t k=0; k<points; ++k) {
                double cy=((positions[2*p+1]+ry[k])-oy)/resolution;
                yc[iy*points+k]=(cy>=0 && cy<h) ? (int)cy : -1;
            }
        }
        xi[p]=ix; yi[p]=iy;
    }
    for (size_t p=0; p<count; ++p) {
        const int *rowx=xc+xi[p]*points, *rowy=yc+yi[p]*points;
        size_t known=0;
        for (size_t k=0; k<points; ++k) {
            float likelihood=0.0f;
            if (rowx[k]>=0 && rowy[k]>=0) {
                size_t cell=(size_t)rowy[k]*w+(size_t)rowx[k];
                if (cells[cell]>=0) { likelihood=field[cell]; ++known; }
            }
            work[k]=likelihood;
        }
        scores[p]=(double)(pairwise_sum(work, points)/(float)points);
        known_ratios[p]=(double)known/(double)points;
    }
    free(xc); free(yc); free(xi); free(yi); free(xs); free(ys);
    return 1;
}

int rvclaw_score_poses(const int8_t *cells, const float *field, int h, int w,
                      double resolution, double ox, double oy, double map_yaw,
                      const double *positions, size_t count,
                      const double *rx, const double *ry, size_t points,
                      float *work, double *scores, double *known_ratios) {
    if (!points || !work || h<=0 || w<=0 || resolution<=0) return -1;
    if (map_yaw==0.0 && score_axis_cached(cells,field,h,w,resolution,ox,oy,
            positions,count,rx,ry,points,work,scores,known_ratios)) return 0;
    const double cosine = cos(map_yaw), sine = sin(map_yaw);
    for (size_t p = 0; p < count; ++p) {
        size_t known = 0;
        for (size_t k = 0; k < points; ++k) {
            double dx = (positions[2*p]+rx[k])-ox;
            double dy = (positions[2*p+1]+ry[k])-oy;
            double x = map_yaw==0.0 ? dx : cosine*dx+sine*dy;
            double y = map_yaw==0.0 ? dy : -sine*dx+cosine*dy;
            double cx = x/resolution, cy = y/resolution;
            float likelihood = 0.0f;
            if (cx>=0 && cy>=0 && cx<w && cy<h) {
                size_t cell = (size_t)cy*w+(size_t)cx;
                if (cells[cell]>=0) { likelihood=field[cell]; ++known; }
            }
            work[k] = likelihood;
        }
        scores[p] = (double)(pairwise_sum(work, points)/(float)points);
        known_ratios[p] = (double)known/(double)points;
    }
    return 0;
}

int rvclaw_ray_stats(const int8_t *cells, int h, int w, double resolution,
                     double ox, double oy, double map_yaw,
                     double origin_x, double origin_y,
                     const double *dx, const double *dy, const double *distances,
                     size_t rays, const double *steps, size_t step_count,
                     double tolerance, uint64_t *stats) {
    if (!stats || resolution<=0 || h<=0 || w<=0) return -1;
    stats[0]=stats[1]=stats[2]=0;
    const double cosine=cos(map_yaw), sine=sin(map_yaw);
    for (size_t k=0; k<rays; ++k) {
        int conflict=0;
        for (size_t j=0; j<step_count && steps[j]<distances[k]-tolerance; ++j) {
            double wx=(origin_x+dx[k]*steps[j])-ox;
            double wy=(origin_y+dy[k]*steps[j])-oy;
            double x=map_yaw==0.0 ? wx : cosine*wx+sine*wy;
            double y=map_yaw==0.0 ? wy : -sine*wx+cosine*wy;
            double cx=x/resolution, cy=y/resolution;
            int state=-1;
            if (cx>=0 && cy>=0 && cx<w && cy<h)
                state=cells[(size_t)cy*w+(size_t)cx];
            if (state==100) conflict=1;
            if (state<0) ++stats[1];
            ++stats[2];
        }
        stats[0]+=conflict;
    }
    return 0;
}
