import type { ClientCmd, ServerMsg, BinaryFrameKind } from '@/types';

type TextHandler = (msg: ServerMsg) => void;
type BinaryHandler = (data: ArrayBuffer, kind: BinaryFrameKind) => void;

export interface WsClientOpts {
  url?: string;
  onText: TextHandler;
  onBinary: BinaryHandler;
  onStateChange?: (state: 'connecting' | 'open' | 'closed') => void;
}

const RECONNECT_INITIAL_MS = 500;
const RECONNECT_MAX_MS = 15000;

export class WsClient {
  private ws: WebSocket | null = null;
  private reconnectMs = RECONNECT_INITIAL_MS;
  private closed = false;
  private iqSubscribed = false;

  constructor(private opts: WsClientOpts) {
    this.connect();
  }

  send(cmd: ClientCmd) {
    if (this.ws?.readyState !== WebSocket.OPEN) return;
    this.ws.send(JSON.stringify(cmd));
    if (cmd.cmd === 'subscribe_iq') this.iqSubscribed = true;
    if (cmd.cmd === 'unsubscribe_iq') this.iqSubscribed = false;
  }

  close() {
    this.closed = true;
    this.ws?.close();
  }

  private connect() {
    const url =
      this.opts.url ??
      `${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`;

    this.opts.onStateChange?.('connecting');
    const ws = new WebSocket(url);
    ws.binaryType = 'arraybuffer';
    this.ws = ws;

    ws.onopen = () => {
      this.reconnectMs = RECONNECT_INITIAL_MS;
      this.opts.onStateChange?.('open');
    };

    ws.onmessage = (ev) => {
      if (typeof ev.data === 'string') {
        try {
          const msg = JSON.parse(ev.data) as ServerMsg;
          this.opts.onText(msg);
        } catch {
          // malformed — ignore silently; the UI shouldn't die on a bad frame
        }
        return;
      }
      if (ev.data instanceof ArrayBuffer) {
        // Implicit routing matches the existing C protocol: the IQ subscription
        // state tells us what a binary frame represents.
        this.opts.onBinary(ev.data, this.iqSubscribed ? 'audio' : 'fft');
      }
    };

    ws.onclose = () => {
      this.opts.onStateChange?.('closed');
      if (this.closed) return;
      setTimeout(() => this.connect(), this.reconnectMs);
      this.reconnectMs = Math.min(this.reconnectMs * 2, RECONNECT_MAX_MS);
    };

    ws.onerror = () => ws.close();
  }
}
