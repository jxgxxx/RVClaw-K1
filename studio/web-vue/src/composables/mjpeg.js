// The camera supplies length-delimited multipart JPEGs. Retain only an
// incomplete part; callers replace queued frames instead of building a backlog.
export class MjpegFrames {
  constructor(onFrame) {
    this.onFrame = onFrame;
    this.buffer = new Uint8Array(0);
    this.length = null;
    this.decoder = new TextDecoder();
  }

  push(chunk) {
    if (this.buffer.length + chunk.length > 5 * 1024 * 1024) throw new Error('视频帧过大');
    const combined = new Uint8Array(this.buffer.length + chunk.length);
    combined.set(this.buffer);
    combined.set(chunk, this.buffer.length);
    this.buffer = combined;
    while (true) {
      if (this.length === null) {
        let end = -1;
        for (let i = 0; i + 3 < this.buffer.length; i++) {
          if (this.buffer[i] === 13 && this.buffer[i + 1] === 10 && this.buffer[i + 2] === 13 && this.buffer[i + 3] === 10) { end = i; break; }
        }
        if (end < 0) {
          if (this.buffer.length > 8192) throw new Error('视频帧头无效');
          return;
        }
        const header = this.decoder.decode(this.buffer.subarray(0, end));
        const match = /^Content-Length:\s*(\d+)\s*$/im.exec(header);
        const length = Number(match?.[1]);
        if (!Number.isSafeInteger(length) || length < 4 || length > 4 * 1024 * 1024) throw new Error('视频帧长度无效');
        this.length = length;
        this.buffer = this.buffer.subarray(end + 4);
      }
      if (this.buffer.length < this.length) return;
      const frame = this.buffer.slice(0, this.length);
      this.buffer = this.buffer.subarray(this.length);
      this.length = null;
      if (frame[0] !== 255 || frame[1] !== 216 || frame.at(-2) !== 255 || frame.at(-1) !== 217) throw new Error('视频帧不完整');
      this.onFrame(frame);
    }
  }
}
