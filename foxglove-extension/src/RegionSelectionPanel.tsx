import {
  MessageEvent,
  PanelExtensionContext,
} from "@foxglove/extension";
import { ReactElement, useEffect, useLayoutEffect, useState } from "react";
import { createRoot } from "react-dom/client";

import "./styles.css";

type Point = { x: number; y: number };
type PanelState = { globalFrame?: string; regionTopic?: string };

const POSE_ARRAY_SCHEMA = "geometry_msgs/msg/PoseArray";
const DEFAULT_FRAME = "map";
const DEFAULT_TOPIC = "/region";

function polygonArea(points: Point[]): number {
  return points.reduce(
    (area, point, index) => {
      const next = points[(index + 1) % points.length]!;
      return area + point.x * next.y - next.x * point.y;
    },
    0,
  ) / 2;
}

function orientation(a: Point, b: Point, c: Point): number {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

function segmentsIntersect(a: Point, b: Point, c: Point, d: Point): boolean {
  const abC = orientation(a, b, c);
  const abD = orientation(a, b, d);
  const cdA = orientation(c, d, a);
  const cdB = orientation(c, d, b);
  return abC * abD < 0 && cdA * cdB < 0;
}

function isSelfIntersecting(points: Point[]): boolean {
  for (let index = 0; index < points.length; index += 1) {
    const next = (index + 1) % points.length;
    for (let other = index + 1; other < points.length; other += 1) {
      const otherNext = (other + 1) % points.length;
      if (index === otherNext || next === other) {
        continue;
      }
      if (segmentsIntersect(points[index]!, points[next]!, points[other]!, points[otherNext]!)) {
        return true;
      }
    }
  }
  return false;
}

function validateRegion(frame: string, points: Point[], globalFrame: string): string | undefined {
  if (frame.trim() !== globalFrame) {
    return `Frame must be "${globalFrame}".`;
  }
  if (points.length < 3) {
    return "Select at least three vertices.";
  }
  if (Math.abs(polygonArea(points)) < 1e-6) {
    return "Vertices must enclose a non-zero area.";
  }
  if (isSelfIntersecting(points)) {
    return "Polygon edges must not cross.";
  }
  return undefined;
}

function toPose(point: Point) {
  return {
    position: { x: point.x, y: point.y, z: 0 },
    orientation: { x: 0, y: 0, z: 0, w: 1 },
  };
}

function messageToPoints(event: MessageEvent): Point[] | undefined {
  const message = event.message as { poses?: Array<{ position?: Point }> };
  if (!message.poses) {
    return undefined;
  }
  return message.poses.flatMap((pose) =>
    pose.position && Number.isFinite(pose.position.x) && Number.isFinite(pose.position.y)
      ? [{ x: pose.position.x, y: pose.position.y }]
      : [],
  );
}

function RegionSelectionPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const initialState = (context.initialState ?? {}) as PanelState;
  const [globalFrame, setGlobalFrame] = useState(initialState.globalFrame ?? DEFAULT_FRAME);
  const [regionTopic, setRegionTopic] = useState(initialState.regionTopic ?? DEFAULT_TOPIC);
  const [frame, setFrame] = useState(initialState.globalFrame ?? DEFAULT_FRAME);
  const [points, setPoints] = useState<Point[]>([]);
  const [lastPublished, setLastPublished] = useState<Point[]>();
  const [lastObserved, setLastObserved] = useState<Point[]>();
  const [error, setError] = useState<string>();
  const [renderDone, setRenderDone] = useState<(() => void)>();

  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      setRenderDone(() => done);
      const regionMessages = (renderState.currentFrame ?? []).filter(
        (event) => event.topic === regionTopic,
      );
      const latest = regionMessages[regionMessages.length - 1];
      if (latest) {
        setLastObserved(messageToPoints(latest));
      }
    };
    context.watch("currentFrame");
    context.subscribe([{ topic: regionTopic }]);
  }, [context, regionTopic]);

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  useEffect(() => {
    context.saveState({ globalFrame, regionTopic });
  }, [context, globalFrame, regionTopic]);

  const addPoint = (point: Point) => {
    setPoints((current) => [...current, point]);
    setError(undefined);
  };

  const publishRegion = async () => {
    const validationError = validateRegion(frame, points, globalFrame);
    if (validationError) {
      setError(validationError);
      return;
    }
    try {
      if (!context.advertise || !context.publish) {
        throw new Error("The connected Foxglove data source does not support publishing.");
      }
      context.advertise(regionTopic, POSE_ARRAY_SCHEMA);
      const message = {
        header: { frame_id: frame, stamp: { sec: 0, nanosec: 0 } },
        poses: points.map(toPose),
      };
      context.publish(regionTopic, message);
      setLastPublished(points);
      setError(undefined);
    } catch (publishError) {
      setError(`Could not publish region: ${String(publishError)}`);
    }
  };

  return (
    <div className="panel">
      <h2>RS1 region selection</h2>
      <p className="hint">
        Click the map area to add vertices. The polygon is published as one
        <code> geometry_msgs/msg/PoseArray</code>.
      </p>
      <div className="settings">
        <label>
          Global frame
          <input value={globalFrame} onChange={(event) => setGlobalFrame(event.target.value)} />
        </label>
        <label>
          Region frame
          <input value={frame} onChange={(event) => setFrame(event.target.value)} />
        </label>
        <label>
          Topic
          <input value={regionTopic} onChange={(event) => setRegionTopic(event.target.value)} />
        </label>
      </div>
      <div
        className="map"
        onClick={(event) => {
          const bounds = event.currentTarget.getBoundingClientRect();
          addPoint({
            x: ((event.clientX - bounds.left) / bounds.width) * 20 - 10,
            y: (1 - (event.clientY - bounds.top) / bounds.height) * 20 - 10,
          });
        }}
        role="application"
        aria-label="Map region selection"
      >
        {points.length > 1 && (
          <svg className="polygon" viewBox="0 0 100 100" aria-hidden="true">
            <polygon
              points={points
                .map(
                  (point) =>
                    `${((point.x + 10) / 20) * 100},${100 - ((point.y + 10) / 20) * 100}`,
                )
                .join(" ")}
            />
          </svg>
        )}
        {points.map((point, index) => (
          <span
            className="vertex"
            key={`${point.x}-${point.y}-${index}`}
            style={{ left: `${((point.x + 10) / 20) * 100}%`, bottom: `${((point.y + 10) / 20) * 100}%` }}
          >
            {index + 1}
          </span>
        ))}
      </div>
      <div className="vertices">
        {points.map((point, index) => (
          <span key={`${point.x}-${point.y}-${index}`}>
            {index + 1}: ({point.x.toFixed(2)}, {point.y.toFixed(2)})
          </span>
        ))}
      </div>
      {error && <div className="error">{error}</div>}
      <div className="actions">
        <button type="button" onClick={() => setPoints([])}>
          Clear
        </button>
        <button type="button" onClick={() => void publishRegion()}>
          Publish region
        </button>
      </div>
      <div className="status">
        <strong>Last published:</strong>{" "}
        {lastPublished ? `${lastPublished.length} vertices on ${frame}` : "none"}
        <br />
        <strong>Last observed:</strong>{" "}
        {lastObserved ? `${lastObserved.length} vertices on ${regionTopic}` : "waiting"}
      </div>
    </div>
  );
}

export function initRegionSelectionPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<RegionSelectionPanel context={context} />);
  return () => root.unmount();
}
