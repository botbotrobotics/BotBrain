'use client';

import { useEffect, useState } from 'react';
import * as ROSLIB from 'roslib';
import { useRobotConnection } from '@/contexts/RobotConnectionContext';
import { CameraType } from '@/types/CameraType';
import { REALSENSE_NODES, RealsenseNode, RealsenseSource } from '@/utils/realsense';
import RobotOffline from './robot-offline';
import RosCameraImg from './ros-camera-img';
import RealsenseCamera, { CameraLoading, CameraViewProps } from './realsense-camera';

const RECHECK_MS = 10000;

/**
 * Only bot_realsense declares stream_enable and depth_min_m / depth_max_m (realsense2_camera
 * does not), so they identify it and give the depth range its grey-depth video encodes.
 */
async function findRealsense(ros: ROSLIB.Ros, node: RealsenseNode): Promise<RealsenseSource | null> {
  // getNodes first: it resolves the namespace, and Humble's rosapi waits 5 s, holding its parameter
  // lock, when asked for a parameter of a node that is not running.
  const nodes = await new Promise<unknown>((resolve, reject) => ros.getNodes(resolve, reject));
  const name = Array.isArray(nodes)
    ? nodes.find((n): n is string => typeof n === 'string' && n.endsWith(`/${node}`))
    : undefined;
  if (!name) return null;

  // rosapi param names are `<absolute node name>:<parameter>`; values arrive JSON-decoded.
  const param = (key: string) =>
    new Promise<unknown>((resolve, reject) => new ROSLIB.Param({ ros, name: `${name}:${key}` }).get(resolve, reject));
  if ((await param('stream_enable')) !== true) return null;
  const [minM, maxM] = await Promise.all([param('depth_min_m'), param('depth_max_m')]);
  if (typeof minM !== 'number' || typeof maxM !== 'number' || !(minM > 0 && maxM > minM && Number.isFinite(maxM))) {
    console.warn(`${name}: depth_min_m=${minM} depth_max_m=${maxM} is not a depth range, showing the ROS feed`);
    return null;
  }
  return { node, nodeName: name, depthRange: { minM, maxM } };
}

/**
 * The bot_realsense camera serving this slot, null if there is none, undefined while asking rosapi.
 * Without one it asks again every RECHECK_MS, which covers robots that are still booting.
 */
function useRealsenseSource(cameraType: CameraType | undefined): RealsenseSource | null | undefined {
  const { ros, online } = useRobotConnection().connection;
  const node = cameraType && REALSENSE_NODES[cameraType];
  const [source, setSource] = useState<RealsenseSource | null>();

  useEffect(() => {
    if (!node || !ros || !online) return;
    let timer: number | undefined;
    let cancelled = false;
    const check = async () => {
      const found = await findRealsense(ros, node).catch(() => null);
      if (cancelled) return;
      setSource(found);
      if (!found) timer = window.setTimeout(check, RECHECK_MS);
    };
    check();
    return () => {
      cancelled = true;
      window.clearTimeout(timer);
      setSource(undefined);
    };
  }, [node, ros, online]);

  return node ? source : null;
}

/** WebRTC for a slot served by a streaming bot_realsense node, RosCameraImg for everything else. */
export default function CameraFeed(props: CameraViewProps) {
  const { connection } = useRobotConnection();
  const source = useRealsenseSource(props.topicName ? undefined : props.cameraType);

  if (!connection.online || !connection.url) {
    return <RobotOffline msg={props.offlineMsg} useBorder />;
  }
  if (source === undefined) return <CameraLoading />;
  if (source === null) return <RosCameraImg {...props} />;
  return <RealsenseCamera {...props} source={source} rosbridgeUrl={connection.url} />;
}
