// A pose correction is not a travelled segment. Keep every actual displayed
// coordinate, but do not join points across a gap or an impossible jump.
export function appendLiveTrack(track, pose) {
  const previous=track.at(-1),dt=pose.sampled_at-(previous?.sampled_at||0);
  const distance=previous?Math.hypot(previous.x-pose.x,previous.y-pose.y):0;
  if(previous&&dt<=0)return track;
  const breakBefore=!previous||(dt>4&&distance>.10)||distance>dt*1.0+.15;
  if(previous&&!breakBefore&&distance<=.03)return [...track.slice(0,-1),{...previous,sampled_at:pose.sampled_at}];
  return [...track.slice(-2047),{x:pose.x,y:pose.y,sampled_at:pose.sampled_at,breakBefore}];
}
