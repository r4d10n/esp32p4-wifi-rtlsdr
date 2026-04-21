// uint8 interleaved IQ (I0,Q0,I1,Q1,…) from the ESP DDC → two Float32Arrays
// in the range [-1, 1], DC-removed per block.

export interface IqBlock {
  i: Float32Array;
  q: Float32Array;
}

const SCALE = 1 / 127.5;
const BIAS = 127.5;

export function decodeIq(buf: ArrayBuffer): IqBlock {
  const u8 = new Uint8Array(buf);
  const n = u8.length >> 1;
  const i = new Float32Array(n);
  const q = new Float32Array(n);
  let iSum = 0, qSum = 0;
  for (let k = 0; k < n; k++) {
    const iv = (u8[k * 2] - BIAS) * SCALE;
    const qv = (u8[k * 2 + 1] - BIAS) * SCALE;
    i[k] = iv; q[k] = qv;
    iSum += iv; qSum += qv;
  }
  // Running DC removal — cheap and usually good enough after the DDC
  // has already centered the channel; catches residual imbalance.
  const iMean = iSum / n;
  const qMean = qSum / n;
  if (iMean !== 0 || qMean !== 0) {
    for (let k = 0; k < n; k++) { i[k] -= iMean; q[k] -= qMean; }
  }
  return { i, q };
}
