'use client';

import { useCallback, useSyncExternalStore } from 'react';

const RETRY_MIN_MS = 1000;
const RETRY_MAX_MS = 10000;
// After a 4xx each new offer still makes MediaMTX retry its sourceOnDemand pull, so ask rarely.
const REJECTED_RETRY_MS = 60000;
// MediaMTX may take up to its sourceOnDemandStartTimeout (10 s) to answer while the robot starts encoding.
const NEGOTIATION_TIMEOUT_MS = 15000;
const ICE_GATHER_MS = 1000;
// Chrome reports `failed` ~10 s after `disconnected`; Wi-Fi blips recover, ended sessions never do.
const DISCONNECTED_GRACE_MS = 2000;
const DELETE_TIMEOUT_MS = 2000;
// A stopped node freezes the last frame and Chrome notices only ~7 s later; STALL_MS ends it sooner.
const STATS_POLL_MS = 500;
const STALL_MS = 2000;
// Covers a cockpit breakpoint switch: the copy that gains layout joins after its lifecycle check.
const LINGER_MS = 3000;

/** Why a WHEP attempt or session ended. */
export type WhepFailure =
  /** The offer POST got no HTTP answer: server down, wrong host or port, firewall, or timeout. */
  | { kind: 'unreachable' }
  /** The server answered the offer with something other than 201 Created. */
  | { kind: 'http'; status: number }
  /** Signalling worked, but the WebRTC media connection failed or dropped. */
  | { kind: 'media' }
  /** Connected, but no video arrived for STALL_MS: the robot stopped sending. */
  | { kind: 'stalled' };

/** What views of one WHEP endpoint show. `attempts` counts consecutive failed attempts. */
export interface WhepState {
  stream: MediaStream | null;
  failure: WhepFailure | null;
  attempts: number;
}

const IDLE: WhepState = { stream: null, failure: null, attempts: 0 };

class WhepError extends Error {
  constructor(readonly failure: WhepFailure, message: string) {
    super(message);
  }
}

interface WhepSession {
  pc: RTCPeerConnection;
  abort: AbortController;
  resource: string | null;
  graceTimer?: number;
  statsTimer?: number;
}

/** While not connected the disconnect grace decides, not the stall timer. */
function watchProgress(pc: RTCPeerConnection, onFlowing: () => void, onStall: () => void): number {
  let progress = 0;
  let changedAt = performance.now();
  return window.setInterval(async () => {
    const report = await pc.getStats().catch(() => null);
    let now = 0;
    report?.forEach((stats) => {
      if (stats.type === 'inbound-rtp' && stats.kind === 'video') now = (stats.bytesReceived ?? 0) + (stats.framesDecoded ?? 0);
    });
    if (now > progress) {
      progress = now;
      changedAt = performance.now();
      onFlowing();
    } else if (pc.connectionState !== 'connected') {
      changedAt = performance.now();
    } else if (performance.now() - changedAt >= STALL_MS) {
      onStall();
    }
  }, STATS_POLL_MS);
}

// DELETE first: MediaMTX drops a session as soon as its peer connection closes, and DELETE then 404s.
function endSession(session: WhepSession) {
  const { pc, abort, resource, graceTimer, statsTimer } = session;
  window.clearTimeout(graceTimer);
  window.clearInterval(statsTimer);
  abort.abort();
  if (!resource) {
    pc.close();
    return;
  }
  fetch(resource, { method: 'DELETE', keepalive: true, signal: AbortSignal.timeout(DELETE_TIMEOUT_MS) })
    .catch(() => {})
    .finally(() => pc.close());
}

/**
 * Plays one WHEP endpoint (receive-only video) until stopped, with at most one session at a time.
 * onStream gets the remote stream once video bytes arrive. onError gets each failed attempt or
 * dropped or stalled session with the count of consecutive failures; the last stream is dead by
 * then. Retries back off from 1 s to 10 s, or wait 60 s after a 4xx. The returned stop function
 * aborts an offer in flight and DELETEs an established session.
 */
export function playWhep(
  url: string,
  onStream: (stream: MediaStream) => void,
  onError: (failure: WhepFailure, attempts: number) => void
): () => void {
  let session: WhepSession | null = null;
  let stopped = false;
  let retryTimer: number | undefined;
  let attempts = 0;

  const close = () => {
    if (!session) return;
    endSession(session);
    session = null;
  };

  const fail = (failure: WhepFailure, reason: unknown) => {
    close();
    if (stopped) return;
    attempts++;
    const rejected = failure.kind === 'http' && failure.status >= 400 && failure.status < 500;
    const delay = rejected ? REJECTED_RETRY_MS : Math.min(RETRY_MIN_MS * 2 ** (attempts - 1), RETRY_MAX_MS);
    console.warn(`WHEP ${url} failed (attempt ${attempts}), retrying in ${delay} ms:`, reason);
    onError(failure, attempts);
    retryTimer = window.setTimeout(connect, delay);
  };

  const connect = async () => {
    const abort = new AbortController();
    const negotiationTimer = window.setTimeout(() => abort.abort(), NEGOTIATION_TIMEOUT_MS);
    let own: WhepSession | null = null;
    try {
      const pc = new RTCPeerConnection();
      const current: WhepSession = { pc, abort, resource: null };
      own = current;
      session = current;
      pc.addTransceiver('video', { direction: 'recvonly' });
      // ontrack fires long before any frame; views get the stream only once video flows.
      let incoming: MediaStream | null = null;
      pc.ontrack = (event) => {
        incoming = event.streams[0] ?? new MediaStream([event.track]);
      };
      pc.onconnectionstatechange = () => {
        if (session !== current) return;
        window.clearTimeout(current.graceTimer);
        switch (pc.connectionState) {
          case 'connected':
            current.statsTimer ??= watchProgress(
              pc,
              () => {
                attempts = 0;
                if (incoming && session === current) onStream(incoming);
                incoming = null;
              },
              () => {
                if (session === current) fail({ kind: 'stalled' }, `no video for ${STALL_MS} ms`);
              }
            );
            break;
          case 'disconnected':
            current.graceTimer = window.setTimeout(() => fail({ kind: 'media' }, 'connection lost'), DISCONNECTED_GRACE_MS);
            break;
          case 'failed':
          case 'closed':
            fail({ kind: 'media' }, `connection ${pc.connectionState}`);
            break;
        }
      };

      await pc.setLocalDescription(await pc.createOffer());
      // Non-trickle: send the offer once host candidates are in (instant on a LAN), or after a second.
      await new Promise<void>((resolve) => {
        const timer = window.setTimeout(resolve, ICE_GATHER_MS);
        const check = () => {
          if (pc.iceGatheringState !== 'complete') return;
          window.clearTimeout(timer);
          resolve();
        };
        pc.onicegatheringstatechange = check;
        check();
      });
      if (session !== current) return;

      let response: Response;
      let answer: string;
      try {
        response = await fetch(url, {
          method: 'POST',
          headers: { 'Content-Type': 'application/sdp' },
          body: pc.localDescription?.sdp,
          signal: abort.signal,
        });
        answer = await response.text();
      } catch (error) {
        throw new WhepError({ kind: 'unreachable' }, String(error));
      }
      if (response.status !== 201) {
        throw new WhepError({ kind: 'http', status: response.status }, `HTTP ${response.status}: ${answer}`);
      }
      const location = response.headers.get('Location');
      current.resource = location && new URL(location, url).href;
      if (session !== current) {
        endSession(current);
        return;
      }
      await pc.setRemoteDescription({ type: 'answer', sdp: answer });
    } catch (error) {
      if (own && session !== own) return;
      fail(error instanceof WhepError ? error.failure : { kind: 'media' }, error);
    } finally {
      window.clearTimeout(negotiationTimer);
    }
  };

  connect();
  return () => {
    stopped = true;
    window.clearTimeout(retryTimer);
    close();
  };
}

interface SharedSession {
  state: WhepState;
  listeners: Set<() => void>;
  stop: () => void;
  lingerTimer?: number;
}

/** One session per WHEP URL, shared because cockpit mounts its camera panel once per breakpoint. */
const sessions = new Map<string, SharedSession>();

function join(url: string, listener: () => void): () => void {
  let shared = sessions.get(url);
  if (!shared) {
    const created: SharedSession = { state: IDLE, listeners: new Set(), stop: () => {} };
    const publish = (state: WhepState) => {
      created.state = state;
      created.listeners.forEach((notify) => notify());
    };
    sessions.set(url, created);
    created.stop = playWhep(
      url,
      (stream) => publish({ stream, failure: null, attempts: 0 }),
      (failure, attempts) => publish({ stream: null, failure, attempts })
    );
    shared = created;
  }
  const joined = shared;
  window.clearTimeout(joined.lingerTimer);
  joined.listeners.add(listener);
  return () => {
    joined.listeners.delete(listener);
    if (joined.listeners.size > 0) return;
    const end = () => {
      if (joined.listeners.size > 0) return;
      joined.stop();
      sessions.delete(url);
    };
    // A failing session ends at once, so it stops retrying (and waking MediaMTX) immediately.
    if (joined.state.stream) joined.lingerTimer = window.setTimeout(end, LINGER_MS);
    else end();
  };
}

/** The shared session state of a WHEP endpoint; idle while url is null. */
export default function useWhepStream(url: string | null): WhepState {
  const subscribe = useCallback((listener: () => void) => (url ? join(url, listener) : () => {}), [url]);
  const snapshot = () => (url ? sessions.get(url)?.state : undefined) ?? IDLE;
  return useSyncExternalStore(subscribe, snapshot, () => IDLE);
}
