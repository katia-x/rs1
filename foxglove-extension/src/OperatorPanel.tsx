import { MessageEvent, PanelExtensionContext } from "@foxglove/extension";
import { ReactElement, useLayoutEffect, useState } from "react";
import type { MouseEvent } from "react";
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
const BOOL_SCHEMA = "std_msgs/msg/Bool";

function finitePoint(point: unknown): point is Point {
  const value = point as Point | undefined;
  return value !== undefined && Number.isFinite(value.x) && Number.isFinite(value.y);
}

function regionPoints(message: unknown): Point[] {
  const poses = (message as { poses?: Array<{ position?: Point }> }).poses ?? [];
  return poses.flatMap((pose) => (finitePoint(pose.position) ? [pose.position] : []));
}

function project(point: Point): { x: number; y: number } {
  return {
    x: ((point.x + 10) / 20) * 100,
    y: ((10 - point.y) / 20) * 100,
  };
}

function unproject(event: MouseEvent<SVGSVGElement>): Point | undefined {
  const bounds = event.currentTarget.getBoundingClientRect();
  if (bounds.width <= 0 || bounds.height <= 0) {
    return undefined;
  }
  return {
    x: ((event.clientX - bounds.left) / bounds.width) * 20 - 10,
    y: (1 - (event.clientY - bounds.top) / bounds.height) * 20 - 10,
  };
}

function rectangleFromCorners(corners: Point[]): Point[] {
  if (corners.length !== 2) {
    return corners;
  }
  const [first, second] = corners;
  return [
    { x: first.x, y: first.y },
    { x: second.x, y: first.y },
    { x: second.x, y: second.y },
    { x: first.x, y: second.y },
  ];
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
    if (draftRegion.length !== 2) {
      setError("Select two opposite corners for the rectangular region.");
      return;
    }
    const rectangle = rectangleFromCorners(draftRegion);
    if (rectangle[0]!.x === rectangle[2]!.x || rectangle[0]!.y === rectangle[2]!.y) {
      setError("The two corners must form a non-zero rectangle.");
      return;
    }
    try {
      if (!context.advertise || !context.publish) {
        throw new Error("The connected Foxglove data source does not support publishing.");
      }
      context.advertise(regionTopic, POSE_ARRAY_SCHEMA);
      context.publish(regionTopic, {
        header: { frame_id: globalFrame, stamp: { sec: 0, nanosec: 0 } },
        poses: rectangle.map((point) => ({
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
      <div className="operator-help">Click two opposite corners to define a rectangular region.</div>
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
        onClick={(event) => {
          const point = unproject(event);
          if (!point) {
            return;
          }
          setError(undefined);
          setDraftRegion((corners) => corners.length >= 2 ? [point] : [...corners, point]);
        }}
        role="application"
        aria-label="Wombat top-down map"
      >
        {region.length > 2 && <polygon className="region-overlay" points={region.map((point) => {
          const projected = project(point);
          return `${projected.x},${projected.y}`;
        }).join(" ")} />}
        {draftRegion.length > 0 && <polygon className="draft-region" points={rectangleFromCorners(draftRegion).map((point) => {
          const projected = project(point);
          return `${projected.x},${projected.y}`;
        }).join(" ")} />}
        {weeds.map((point, index) => {
          const projected = project(point);
          return <circle className="weed-point" key={`weed-${index}`} cx={projected.x} cy={projected.y} r="1.3" />;
        })}
        {target && <circle className="target-point" cx={project(target).x} cy={project(target).y} r="2" />}
        {wombat && <rect className="wombat-point" x={project(wombat).x - 1.5} y={project(wombat).y - 1.5} width="3" height="3" />}
      </svg>
      <div className="operator-actions">
        <button type="button" onClick={() => setDraftRegion([])}>Clear draft</button>
        <button type="button" onClick={publishRegion}>Publish region</button>
      </div>
      {error && <div className="error">{error}</div>}
      <div className="operator-diagnostics">
        <span>Region: {region.length} vertices</span>
        <span>Draft: {draftRegion.length === 2 ? "rectangle ready" : `${draftRegion.length}/2 corners`}</span>
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
