import { MessageEvent, PanelExtensionContext } from "@foxglove/extension";
import { ReactElement, useLayoutEffect, useState } from "react";
import { createRoot } from "react-dom/client";

import "./styles.css";

type Point = { x: number; y: number };
type PanelState = {
  globalFrame?: string;
  regionTopic?: string;
  estimateTopic?: string;
  targetTopic?: string;
  tfTopic?: string;
  overrideTopic?: string;
  overrideCommandTopic?: string;
};
type Marker = {
  action?: number;
  header?: { frame_id?: string; stamp?: { sec?: number; nanosec?: number } };
  pose?: { position?: Point };
  id?: number;
};
type FrameEvent = MessageEvent & { topic: string };

const MARKER_ADD = 0;
const MARKER_DELETE = 2;
const MARKER_DELETEALL = 3;
const POSE_ARRAY_SCHEMA = "geometry_msgs/msg/PoseArray";
const MARKER_ARRAY_SCHEMA = "visualization_msgs/msg/MarkerArray";
const BOOL_SCHEMA = "std_msgs/msg/Bool";

function finitePoint(point: unknown): point is Point {
  const value = point as Point | undefined;
  return value !== undefined && Number.isFinite(value.x) && Number.isFinite(value.y);
}

function regionPoints(message: unknown): Point[] {
  const poses = (message as { poses?: Array<{ position?: Point }> }).poses ?? [];
  return poses.flatMap((pose) => (finitePoint(pose.position) ? [pose.position] : []));
}

function polygonArea(points: Point[]): number {
  return points.reduce((area, point, index) => {
    const next = points[(index + 1) % points.length]!;
    return area + point.x * next.y - next.x * point.y;
  }, 0) / 2;
}

function orientation(a: Point, b: Point, c: Point): number {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

function selfIntersects(points: Point[]): boolean {
  for (let index = 0; index < points.length; index += 1) {
    const next = (index + 1) % points.length;
    for (let other = index + 1; other < points.length; other += 1) {
      const otherNext = (other + 1) % points.length;
      if (index === otherNext || next === other) {
        continue;
      }
      const abC = orientation(points[index]!, points[next]!, points[other]!);
      const abD = orientation(points[index]!, points[next]!, points[otherNext]!);
      const cdA = orientation(points[other]!, points[otherNext]!, points[index]!);
      const cdB = orientation(points[other]!, points[otherNext]!, points[next]!);
      if (abC * abD < 0 && cdA * cdB < 0) {
        return true;
      }
    }
  }
  return false;
}

function project(point: Point): { left: string; top: string } {
  return {
    left: `${((point.x + 10) / 20) * 100}%`,
    top: `${((10 - point.y) / 20) * 100}%`,
  };
}

function unproject(event: React.MouseEvent<SVGSVGElement>): Point {
  const bounds = event.currentTarget.getBoundingClientRect();
  return {
    x: ((event.clientX - bounds.left) / bounds.width) * 20 - 10,
    y: (1 - (event.clientY - bounds.top) / bounds.height) * 20 - 10,
  };
}

function OperatorPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const initialState = (context.initialState ?? {}) as PanelState;
  const [globalFrame, setGlobalFrame] = useState(initialState.globalFrame ?? "map");
  const [regionTopic, setRegionTopic] = useState(initialState.regionTopic ?? "/region");
  const [estimateTopic, setEstimateTopic] = useState(
    initialState.estimateTopic ?? "/husky1/weed_estimate_markers",
  );
  const [targetTopic, setTargetTopic] = useState(
    initialState.targetTopic ?? "/husky1/target_weed_marker",
  );
  const [tfTopic, setTfTopic] = useState(initialState.tfTopic ?? "/husky1/tf");
  const [overrideTopic, setOverrideTopic] = useState(
    initialState.overrideTopic ?? "/husky1/manual_override",
  );
  const [overrideCommandTopic, setOverrideCommandTopic] = useState(
    initialState.overrideCommandTopic ?? "/husky1/manual_override_command",
  );
  const [region, setRegion] = useState<Point[]>([]);
  const [draftRegion, setDraftRegion] = useState<Point[]>([]);
  const [weeds, setWeeds] = useState<Point[]>([]);
  const [target, setTarget] = useState<Point>();
  const [wombat, setWombat] = useState<Point>();
  const [override, setOverride] = useState(false);
  const [lastMarkerTime, setLastMarkerTime] = useState<number>();
  const [lastTfTime, setLastTfTime] = useState<number>();
  const [lastOverrideTime, setLastOverrideTime] = useState<number>();
  const [error, setError] = useState<string>();

  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      const events = (renderState.currentFrame ?? []) as FrameEvent[];
      const latest = (topic: string) => {
        const matching = events.filter((event) => event.topic === topic);
        return matching[matching.length - 1];
      };
      const regionEvent = latest(regionTopic);
      if (regionEvent) {
        const message = regionEvent.message as { header?: { frame_id?: string } };
        if (!message.header?.frame_id || message.header.frame_id === globalFrame) {
          setRegion(regionPoints(regionEvent.message));
        }
      }

      const estimateEvent = latest(estimateTopic);
      if (estimateEvent) {
        const markers = ((estimateEvent.message as { markers?: Marker[] }).markers ?? []);
        const visible = markers.filter(
          (marker) => marker.action !== MARKER_DELETEALL && marker.action !== MARKER_DELETE &&
            (!marker.header?.frame_id || marker.header.frame_id === globalFrame) &&
            finitePoint(marker.pose?.position),
        );
        setWeeds(visible.map((marker) => marker.pose!.position!));
        setLastMarkerTime(Date.now());
      }

      const targetEvent = latest(targetTopic);
      if (targetEvent) {
        const marker = targetEvent.message as Marker;
        setTarget(
          marker.action === MARKER_ADD &&
            (!marker.header?.frame_id || marker.header.frame_id === globalFrame) &&
            finitePoint(marker.pose?.position)
            ? marker.pose.position
            : undefined,
        );
      }

      const tfEvent = latest(tfTopic);
      if (tfEvent) {
        const transforms = (
          tfEvent.message as {
            transforms?: Array<{
              header?: { frame_id?: string };
              child_frame_id?: string;
              transform?: { translation?: Point };
            }>;
          }
        ).transforms ?? [];
        const transform = transforms.find(
          (value) => value.header?.frame_id === globalFrame &&
            value.child_frame_id === "husky1_base_link" &&
            finitePoint(value.transform?.translation),
        );
        if (transform) {
          setWombat(transform.transform!.translation);
          setLastTfTime(Date.now());
        }
      }

      const overrideEvent = latest(overrideTopic);
      if (overrideEvent) {
        setOverride(Boolean((overrideEvent.message as { data?: boolean }).data));
        setLastOverrideTime(Date.now());
      }
      done();
    };
    context.watch("currentFrame");
    context.subscribe([
      { topic: regionTopic },
      { topic: estimateTopic },
      { topic: targetTopic },
      { topic: tfTopic },
      { topic: overrideTopic },
    ]);
  }, [context, estimateTopic, globalFrame, overrideTopic, regionTopic, targetTopic, tfTopic]);

  const saveState = () =>
    context.saveState({
      globalFrame,
      regionTopic,
      estimateTopic,
      targetTopic,
      tfTopic,
      overrideTopic,
      overrideCommandTopic,
    });

  const publishRegion = () => {
    if (draftRegion.length < 3) {
      setError("Select at least three region vertices.");
      return;
    }
    if (Math.abs(polygonArea(draftRegion)) < 1e-6 || selfIntersects(draftRegion)) {
      setError("Region vertices must form a non-crossing polygon with non-zero area.");
      return;
    }
    try {
      if (!context.advertise || !context.publish) {
        throw new Error("The connected Foxglove data source does not support publishing.");
      }
      context.advertise(regionTopic, POSE_ARRAY_SCHEMA);
      context.publish(regionTopic, {
        header: { frame_id: globalFrame, stamp: { sec: 0, nanosec: 0 } },
        poses: draftRegion.map((point) => ({
          position: { x: point.x, y: point.y, z: 0 },
          orientation: { x: 0, y: 0, z: 0, w: 1 },
        })),
      });
      setError(undefined);
    } catch (publishError) {
      setError(`Could not publish region: ${String(publishError)}`);
    }
  };

  const setManualOverride = (enabled: boolean) => {
    try {
      if (!context.advertise || !context.publish) {
        throw new Error("The connected Foxglove data source does not support publishing.");
      }
      context.advertise(overrideCommandTopic, BOOL_SCHEMA);
      context.publish(overrideCommandTopic, { data: enabled });
      setError(undefined);
    } catch (publishError) {
      setError(`Could not request manual override: ${String(publishError)}`);
    }
  };

  const age = (time?: number) => (time === undefined ? "missing" : `${((Date.now() - time) / 1000).toFixed(1)}s ago`);

  return (
    <div className="operator-panel">
      <h2>Wombat operator map</h2>
      <div className="operator-settings">
        <label>Global frame<input value={globalFrame} onChange={(event) => setGlobalFrame(event.target.value)} onBlur={saveState} /></label>
        <label>Region topic<input value={regionTopic} onChange={(event) => setRegionTopic(event.target.value)} onBlur={saveState} /></label>
        <label>Estimate topic<input value={estimateTopic} onChange={(event) => setEstimateTopic(event.target.value)} onBlur={saveState} /></label>
        <label>TF topic<input value={tfTopic} onChange={(event) => setTfTopic(event.target.value)} onBlur={saveState} /></label>
      </div>
      <div className="operator-status">
        <strong>Manual override:</strong>{" "}
        <span className={override ? "status-enabled" : "status-disabled"}>
          {override ? "ENABLED" : "DISABLED"}
        </span>
        <button type="button" onClick={() => setManualOverride(!override)}>
          Request {override ? "automatic" : "manual"} mode
        </button>
        <span>state {age(lastOverrideTime)}</span>
      </div>
      <svg
        className="operator-map"
        viewBox="0 0 100 100"
        onClick={(event) => setDraftRegion((points) => [...points, unproject(event)])}
        role="application"
        aria-label="Wombat top-down map"
      >
        {region.length > 2 && <polygon className="region-overlay" points={region.map((point) => `${project(point).left.replace("%", "")},${project(point).top.replace("%", "")}`).join(" ")} />}
        {draftRegion.length > 1 && <polyline className="draft-region" points={draftRegion.map((point) => `${project(point).left.replace("%", "")},${project(point).top.replace("%", "")}`).join(" ")} />}
        {weeds.map((point, index) => <circle className="weed-point" key={`weed-${index}`} cx={`${((point.x + 10) / 20) * 100}`} cy={`${((10 - point.y) / 20) * 100}`} r="1.3" />)}
        {target && <circle className="target-point" cx={`${((target.x + 10) / 20) * 100}`} cy={`${((10 - target.y) / 20) * 100}`} r="2" />}
        {wombat && <rect className="wombat-point" x={`${((wombat.x + 10) / 20) * 100 - 1.5}`} y={`${((10 - wombat.y) / 20) * 100 - 1.5}`} width="3" height="3" />}
      </svg>
      <div className="operator-actions">
        <button type="button" onClick={() => setDraftRegion([])}>Clear draft</button>
        <button type="button" onClick={publishRegion}>Publish region</button>
      </div>
      {error && <div className="error">{error}</div>}
      <div className="operator-diagnostics">
        <span>Region: {region.length} vertices</span>
        <span>Draft: {draftRegion.length} vertices</span>
        <span>Weeds: {weeds.length}</span>
        <span>Target: {target ? "visible" : "none"}</span>
        <span>Wombat TF: {wombat ? "available" : "missing"} ({age(lastTfTime)})</span>
        <span>Markers: {age(lastMarkerTime)}</span>
      </div>
    </div>
  );
}

export function initOperatorPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<OperatorPanel context={context} />);
  return () => root.unmount();
}
