import { MessageEvent, PanelExtensionContext } from "@foxglove/extension";
import { ReactElement, useLayoutEffect, useRef, useState } from "react";
import type { MouseEvent } from "react";
import { createRoot } from "react-dom/client";

import smallMapUrl from "./assets/small_map.png";
import "./styles.css";

type Point = { x: number; y: number };
type EstopCycle = "idle" | "active" | "released";
type PanelState = {
  globalFrame?: string;
  baseFrame?: string;
  regionTopic?: string;
  estimateTopic?: string;
  permanentTopic?: string;
  targetTopic?: string;
  tfTopic?: string;
  tfStaticTopic?: string;
  gliderTfTopic?: string;
  gliderTfStaticTopic?: string;
  gliderFrame?: string;
  overrideTopic?: string;
  overrideCommandTopic?: string;
  estopTopic?: string;
  estopCommandTopic?: string;
  resumeCommandTopic?: string;
};
type Marker = {
  action?: number;
  header?: { frame_id?: string; stamp?: { sec?: number; nanosec?: number } };
  pose?: { position?: Point };
  id?: number;
};
type FrameEvent = MessageEvent & { topic: string };
type Transform = {
  header?: { frame_id?: string };
  child_frame_id?: string;
  transform?: {
    translation?: Point;
    rotation?: { x?: number; y?: number; z?: number; w?: number };
  };
};
type Rotation = NonNullable<NonNullable<Transform["transform"]>["rotation"]>;

const MARKER_ADD = 0;
const MARKER_DELETE = 2;
const MARKER_DELETEALL = 3;
const POSE_ARRAY_SCHEMA = "geometry_msgs/msg/PoseArray";
const BOOL_SCHEMA = "std_msgs/msg/Bool";
const MAX_TRAIL_POINTS = 1000;
const MIN_TRAIL_DISTANCE = 0.05;

function finitePoint(point: unknown): point is Point {
  const value = point as Point | undefined;
  return (
    value !== undefined && Number.isFinite(value.x) && Number.isFinite(value.y)
  );
}

function regionPoints(message: unknown): Point[] {
  const poses =
    (message as { poses?: Array<{ position?: Point }> }).poses ?? [];
  return poses.flatMap((pose) =>
    finitePoint(pose.position) ? [pose.position] : [],
  );
}

function project(point: Point): { x: number; y: number } {
  return {
    x: ((point.x + 10) / 20) * 100,
    y: ((10 - point.y) / 20) * 100,
  };
}

function unproject(event: MouseEvent<SVGSVGElement>): Point | undefined {
  const svg = event.currentTarget;
  const screenMatrix = svg.getScreenCTM();
  if (!screenMatrix) {
    return undefined;
  }
  const point = new DOMPoint(event.clientX, event.clientY).matrixTransform(
    screenMatrix.inverse(),
  );
  if (!Number.isFinite(point.x) || !Number.isFinite(point.y)) {
    return undefined;
  }
  if (point.x < 0 || point.x > 100 || point.y < 0 || point.y > 100) {
    return undefined;
  }
  return {
    x: (point.x / 100) * 20 - 10,
    y: (1 - point.y / 100) * 20 - 10,
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

function yaw(rotation?: Rotation): number {
  if (
    rotation === undefined ||
    !Number.isFinite(rotation.x) ||
    !Number.isFinite(rotation.y) ||
    !Number.isFinite(rotation.z) ||
    !Number.isFinite(rotation.w)
  ) {
    return 0;
  }
  return Math.atan2(
    2 * (rotation.w! * rotation.z! + rotation.x! * rotation.y!),
    1 - 2 * (rotation.y! * rotation.y! + rotation.z! * rotation.z!),
  );
}

function compose(
  first: Point,
  firstYaw: number,
  second: Point,
  secondYaw: number,
) {
  const cosine = Math.cos(firstYaw);
  const sine = Math.sin(firstYaw);
  return {
    point: {
      x: first.x + cosine * second.x - sine * second.y,
      y: first.y + sine * second.x + cosine * second.y,
    },
    yaw: firstYaw + secondYaw,
  };
}

function resolveTransform(
  transforms: Transform[],
  sourceFrame: string,
  targetFrame: string,
): Point | undefined {
  const normalizeFrame = (frame: string) => frame.replace(/^\/+/, "");
  const normalizedSourceFrame = normalizeFrame(sourceFrame);
  const normalizedTargetFrame = normalizeFrame(targetFrame);
  const queue: Array<{ frame: string; point: Point; yaw: number }> = [
    { frame: normalizedSourceFrame, point: { x: 0, y: 0 }, yaw: 0 },
  ];
  const visited = new Set([normalizedSourceFrame]);
  while (queue.length > 0) {
    const current = queue.shift()!;
    if (current.frame === normalizedTargetFrame) {
      return current.point;
    }
    for (const transform of transforms) {
      const parent = transform.header?.frame_id
        ? normalizeFrame(transform.header.frame_id)
        : undefined;
      const child = transform.child_frame_id
        ? normalizeFrame(transform.child_frame_id)
        : undefined;
      const translation = transform.transform?.translation;
      if (
        !parent ||
        !child ||
        !finitePoint(translation) ||
        (parent !== current.frame && child !== current.frame)
      ) {
        continue;
      }
      const transformYaw = yaw(transform.transform?.rotation);
      const nextFrame = parent === current.frame ? child : parent;
      if (visited.has(nextFrame)) {
        continue;
      }
      const next =
        parent === current.frame
          ? compose(current.point, current.yaw, translation, transformYaw)
          : compose(
              current.point,
              current.yaw,
              {
                x:
                  -Math.cos(transformYaw) * translation.x -
                  Math.sin(transformYaw) * translation.y,
                y:
                  Math.sin(transformYaw) * translation.x -
                  Math.cos(transformYaw) * translation.y,
              },
              -transformYaw,
            );
      visited.add(nextFrame);
      queue.push({ frame: nextFrame, point: next.point, yaw: next.yaw });
    }
  }
  return undefined;
}

function OperatorPanel({
  context,
}: {
  context: PanelExtensionContext;
}): ReactElement {
  const initialState = (context.initialState ?? {}) as PanelState;
  const [globalFrame, setGlobalFrame] = useState(
    initialState.globalFrame ?? "map",
  );
  const [baseFrame, setBaseFrame] = useState(
    initialState.baseFrame ?? "husky1_base_link",
  );
  const [regionTopic, setRegionTopic] = useState(
    initialState.regionTopic ?? "/region",
  );
  const [estimateTopic, setEstimateTopic] = useState(
    initialState.estimateTopic ?? "/husky1/weed_estimate_markers",
  );
  const [permanentTopic, setPermanentTopic] = useState(
    initialState.permanentTopic ?? "/husky1/permanent_weed_exclusions",
  );
  const [targetTopic, setTargetTopic] = useState(
    initialState.targetTopic ?? "/husky1/target_weed_marker",
  );
  const [tfTopic, setTfTopic] = useState(initialState.tfTopic ?? "/husky1/tf");
  const [tfStaticTopic, setTfStaticTopic] = useState(
    initialState.tfStaticTopic ?? "/husky1/tf_static",
  );
  const [gliderTfTopic, setGliderTfTopic] = useState(
    initialState.gliderTfTopic ?? "/parrot1/tf",
  );
  const [gliderTfStaticTopic, setGliderTfStaticTopic] = useState(
    initialState.gliderTfStaticTopic ?? "/parrot1/tf_static",
  );
  const [gliderFrame, setGliderFrame] = useState(
    initialState.gliderFrame ?? "parrot1_base_link",
  );
  const [overrideTopic, setOverrideTopic] = useState(
    initialState.overrideTopic ?? "/husky1/manual_override",
  );
  const [overrideCommandTopic, setOverrideCommandTopic] = useState(
    initialState.overrideCommandTopic ?? "/husky1/manual_override_command",
  );
  const estopTopic = initialState.estopTopic ?? "/husky1/estop";
  const estopCommandTopic =
    initialState.estopCommandTopic ?? "/husky1/estop_command";
  const resumeCommandTopic =
    initialState.resumeCommandTopic ?? "/husky1/resume_command";
  const [region, setRegion] = useState<Point[]>([]);
  const [draftRegion, setDraftRegion] = useState<Point[]>([]);
  const [weeds, setWeeds] = useState<Point[]>([]);
  const [permanentWeeds, setPermanentWeeds] = useState<Point[]>([]);
  const [target, setTarget] = useState<Point>();
  const [wombat, setWombat] = useState<Point>();
  const [glider, setGlider] = useState<Point>();
  const [wombatTrail, setWombatTrail] = useState<Point[]>([]);
  const lastTrailPoint = useRef<Point>();
  const transforms = useRef<Transform[]>([]);
  const [override, setOverride] = useState(false);
  const [estop, setEstop] = useState(false);
  const [estopCycle, setEstopCycle] = useState<EstopCycle>("idle");
  const [lastMarkerTime, setLastMarkerTime] = useState<number>();
  const [lastTfTime, setLastTfTime] = useState<number>();
  const [lastOverrideTime, setLastOverrideTime] = useState<number>();
  const [error, setError] = useState<string>();

  useLayoutEffect(() => {
    lastTrailPoint.current = undefined;
    transforms.current = [];
    setWombatTrail([]);
    context.onRender = (renderState, done) => {
      const events = (renderState.currentFrame ?? []) as FrameEvent[];
      if (renderState.didSeek) {
        transforms.current = [];
        lastTrailPoint.current = undefined;
        setWombat(undefined);
        setGlider(undefined);
        setWombatTrail([]);
      }
      const latest = (topic: string) => {
        const matching = events.filter((event) => event.topic === topic);
        return matching[matching.length - 1];
      };
      const regionEvent = latest(regionTopic);
      if (regionEvent) {
        const message = regionEvent.message as {
          header?: { frame_id?: string };
        };
        if (
          !message.header?.frame_id ||
          message.header.frame_id === globalFrame
        ) {
          setRegion(regionPoints(regionEvent.message));
        }
      }

      const estimateEvent = latest(estimateTopic);
      if (estimateEvent) {
        const markers =
          (estimateEvent.message as { markers?: Marker[] }).markers ?? [];
        const visible = markers.filter(
          (marker) =>
            marker.action !== MARKER_DELETEALL &&
            marker.action !== MARKER_DELETE &&
            (!marker.header?.frame_id ||
              marker.header.frame_id === globalFrame) &&
            finitePoint(marker.pose?.position),
        );
        setWeeds(visible.map((marker) => marker.pose!.position!));
        setLastMarkerTime(Date.now());
      }

      const permanentEvent = latest(permanentTopic);
      if (permanentEvent) {
        const message = permanentEvent.message as {
          header?: { frame_id?: string };
        };
        if (
          !message.header?.frame_id ||
          message.header.frame_id === globalFrame
        ) {
          setPermanentWeeds(regionPoints(permanentEvent.message));
        }
      }

      const targetEvent = latest(targetTopic);
      if (targetEvent) {
        const marker = targetEvent.message as Marker;
        setTarget(
          marker.action === MARKER_ADD &&
            (!marker.header?.frame_id ||
              marker.header.frame_id === globalFrame) &&
            finitePoint(marker.pose?.position)
            ? marker.pose.position
            : undefined,
        );
      }

      for (const topic of [
        tfTopic,
        tfStaticTopic,
        gliderTfTopic,
        gliderTfStaticTopic,
      ]) {
        const tfEvent = latest(topic);
        if (tfEvent) {
          const incoming =
            (tfEvent.message as { transforms?: Transform[] }).transforms ?? [];
          transforms.current = [
            ...transforms.current.filter(
              (known) =>
                !incoming.some(
                  (value) =>
                    value.header?.frame_id === known.header?.frame_id &&
                    value.child_frame_id === known.child_frame_id,
                ),
            ),
            ...incoming,
          ];
        }
      }
      const position = resolveTransform(
        transforms.current,
        globalFrame,
        baseFrame,
      );
      if (position) {
        setWombat(position);
        const previous = lastTrailPoint.current;
        if (
          previous === undefined ||
          Math.hypot(position.x - previous.x, position.y - previous.y) >=
            MIN_TRAIL_DISTANCE
        ) {
          lastTrailPoint.current = position;
          setWombatTrail((trail) =>
            [...trail, position].slice(-MAX_TRAIL_POINTS),
          );
        }
        setLastTfTime(Date.now());
      }
      setGlider(
        resolveTransform(transforms.current, globalFrame, gliderFrame),
      );

      const overrideEvent = latest(overrideTopic);
      if (overrideEvent) {
        setOverride(
          Boolean((overrideEvent.message as { data?: boolean }).data),
        );
        setLastOverrideTime(Date.now());
      }
      const estopEvent = latest(estopTopic);
      if (estopEvent) {
        const active = Boolean(
          (estopEvent.message as { data?: boolean }).data,
        );
        setEstop(active);
        if (active) {
          setEstopCycle("active");
        } else if (estopCycle !== "idle") {
          setEstopCycle("released");
        }
      }
      done();
    };
    context.watch("currentFrame");
    context.subscribe([
      { topic: regionTopic },
      { topic: estimateTopic },
      { topic: permanentTopic },
      { topic: targetTopic },
      { topic: tfTopic },
      { topic: tfStaticTopic },
      { topic: gliderTfTopic },
      { topic: gliderTfStaticTopic },
      { topic: overrideTopic },
      { topic: estopTopic },
    ]);
  }, [
    context,
    estimateTopic,
    permanentTopic,
    globalFrame,
    baseFrame,
    overrideTopic,
    estopTopic,
    estopCycle,
    regionTopic,
    targetTopic,
    tfTopic,
    tfStaticTopic,
    gliderTfTopic,
    gliderTfStaticTopic,
    gliderFrame,
  ]);

  const saveState = () =>
    context.saveState({
      globalFrame,
      baseFrame,
      regionTopic,
      estimateTopic,
      permanentTopic,
      targetTopic,
      tfTopic,
      tfStaticTopic,
      gliderTfTopic,
      gliderTfStaticTopic,
      gliderFrame,
      overrideTopic,
      overrideCommandTopic,
      estopTopic,
      estopCommandTopic,
      resumeCommandTopic,
    });

  const publishRegion = () => {
    if (draftRegion.length !== 2) {
      setError("Select two opposite corners for the rectangular region.");
      return;
    }
    const rectangle = rectangleFromCorners(draftRegion);
    if (
      rectangle[0]!.x === rectangle[2]!.x ||
      rectangle[0]!.y === rectangle[2]!.y
    ) {
      setError("The two corners must form a non-zero rectangle.");
      return;
    }
    try {
      if (!context.advertise || !context.publish) {
        throw new Error(
          "The connected Foxglove data source does not support publishing.",
        );
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

  const publishBool = (
    topic: string,
    value: boolean,
    action: string,
    onPublished?: () => void,
  ) => {
    try {
      if (!context.advertise || !context.publish) {
        throw new Error(
          "The connected Foxglove data source does not support publishing.",
        );
      }
      context.advertise(topic, BOOL_SCHEMA);
      context.publish(topic, { data: value });
      onPublished?.();
      setError(undefined);
    } catch (publishError) {
      setError(`Could not request ${action}: ${String(publishError)}`);
    }
  };

  const age = (time?: number) =>
    time === undefined
      ? "missing"
      : `${((Date.now() - time) / 1000).toFixed(1)}s ago`;

  const toggleEstop = () => {
    const nextEstop = !estop;
    publishBool(estopCommandTopic, nextEstop, "E-stop", () => {
      setEstop(nextEstop);
      setEstopCycle(nextEstop ? "active" : "released");
    });
  };

  return (
    <div className="operator-panel">
      <h2>Wombat operator map</h2>
      <div className="operator-help">
        Click two opposite corners to define a rectangular region.
      </div>
      <div className="operator-settings">
        <label>
          Global frame
          <input
            value={globalFrame}
            onChange={(event) => setGlobalFrame(event.target.value)}
            onBlur={saveState}
          />
        </label>
        <label>
          Base frame
          <input
            value={baseFrame}
            onChange={(event) => setBaseFrame(event.target.value)}
            onBlur={saveState}
          />
        </label>
        <label>
          Region topic
          <input
            value={regionTopic}
            onChange={(event) => setRegionTopic(event.target.value)}
            onBlur={saveState}
          />
        </label>
        <label>
          Estimate topic
          <input
            value={estimateTopic}
            onChange={(event) => setEstimateTopic(event.target.value)}
            onBlur={saveState}
          />
        </label>
        <label>
          Permanent weed topic
          <input
            value={permanentTopic}
            onChange={(event) => setPermanentTopic(event.target.value)}
            onBlur={saveState}
          />
        </label>
        <label>
          TF topic
          <input
            value={tfTopic}
            onChange={(event) => setTfTopic(event.target.value)}
            onBlur={saveState}
          />
        </label>
        <label>
          Static TF topic
          <input
            value={tfStaticTopic}
            onChange={(event) => setTfStaticTopic(event.target.value)}
            onBlur={saveState}
          />
        </label>
        <label>
          Glider TF topic
          <input
            value={gliderTfTopic}
            onChange={(event) => setGliderTfTopic(event.target.value)}
            onBlur={saveState}
          />
        </label>
        <label>
          Glider static TF topic
          <input
            value={gliderTfStaticTopic}
            onChange={(event) => setGliderTfStaticTopic(event.target.value)}
            onBlur={saveState}
          />
        </label>
        <label>
          Glider frame
          <input
            value={gliderFrame}
            onChange={(event) => setGliderFrame(event.target.value)}
            onBlur={saveState}
          />
        </label>
      </div>
      <div className="operator-status">
        <strong>Manual override:</strong>{" "}
        <span className={override ? "status-enabled" : "status-disabled"}>
          {override ? "ENABLED" : "DISABLED"}
        </span>
        <button
          type="button"
          onClick={() => publishBool(overrideCommandTopic, !override, "manual mode")}
        >
          {override ? "Automatic" : "Manual Override"}
        </button>
        <button
          className="estop-button"
          type="button"
          onClick={toggleEstop}
        >
          {estop ? "Release" : "E-stop"}
        </button>
        <button
          className="release-button"
          type="button"
          disabled={estop || estopCycle !== "released"}
          onClick={() =>
            publishBool(resumeCommandTopic, true, "resume")
          }
        >
          Resume
        </button>
        <span>state {age(lastOverrideTime)}</span>
      </div>
      <div className="operator-map-container">
        <svg
          className="operator-map"
          viewBox="0 0 100 100"
          preserveAspectRatio="xMidYMid meet"
          onClick={(event) => {
            const point = unproject(event);
            if (!point) {
              return;
            }
            setError(undefined);
            setDraftRegion((corners) =>
              corners.length >= 2 ? [point] : [...corners, point],
            );
          }}
          role="application"
          aria-label="Wombat top-down map"
        >
          <title>
            Wombat position and trail in the configured global frame
          </title>
          <image
            className="map-background-image"
            href={smallMapUrl}
            x="0"
            y="0"
            width="100"
            height="100"
            preserveAspectRatio="xMidYMid meet"
            aria-hidden="true"
          />
          {region.length > 2 && (
            <polygon
              className="region-overlay"
              points={region
                .map((point) => {
                  const projected = project(point);
                  return `${projected.x},${projected.y}`;
                })
                .join(" ")}
            />
          )}
          {draftRegion.length > 0 && (
            <polygon
              className="draft-region"
              points={rectangleFromCorners(draftRegion)
                .map((point) => {
                  const projected = project(point);
                  return `${projected.x},${projected.y}`;
                })
                .join(" ")}
            />
          )}
          {weeds.map((point, index) => {
            const projected = project(point);
            return (
              <circle
                className="weed-point"
                key={`weed-${index}`}
                cx={projected.x}
                cy={projected.y}
                r="1.3"
              />
            );
          })}
          {permanentWeeds.map((point, index) => {
            const projected = project(point);
            return (
              <circle
                className="permanent-weed-point"
                key={`permanent-weed-${index}`}
                cx={projected.x}
                cy={projected.y}
                r="1.5"
              />
            );
          })}
          {target && (
            <circle
              className="target-point"
              cx={project(target).x}
              cy={project(target).y}
              r="2"
            />
          )}
          {wombatTrail.length > 1 && (
            <polyline
              className="wombat-trail"
              points={wombatTrail
                .map((point) => {
                  const projected = project(point);
                  return `${projected.x},${projected.y}`;
                })
                .join(" ")}
            />
          )}
          {wombat && (
            <rect
              className="wombat-point"
              x={project(wombat).x - 1.5}
              y={project(wombat).y - 1.5}
              width="3"
              height="3"
            />
          )}
          {glider && (
            <circle
              className="glider-point"
              cx={project(glider).x}
              cy={project(glider).y}
              r="1.8"
            />
          )}
        </svg>
      </div>
      <div className="operator-actions">
        <button type="button" onClick={() => setDraftRegion([])}>
          Clear draft
        </button>
        <button
          type="button"
          onClick={() => {
            setWombatTrail(wombat ? [wombat] : []);
            lastTrailPoint.current = wombat;
          }}
        >
          Clear trail
        </button>
        <button type="button" onClick={publishRegion}>
          Publish region
        </button>
      </div>
      {error && <div className="error">{error}</div>}
      <div className="operator-diagnostics">
        <span>Region: {region.length} vertices</span>
        <span>
          Draft:{" "}
          {draftRegion.length === 2
            ? "rectangle ready"
            : `${draftRegion.length}/2 corners`}
        </span>
        <span>Weeds: {weeds.length}</span>
        <span>Permanent weeds: {permanentWeeds.length}</span>
        <span>Target: {target ? "visible" : "none"}</span>
        <span>
          Wombat TF: {wombat ? "available" : "missing"} ({age(lastTfTime)})
        </span>
        <span>Glider TF: {glider ? "available" : "missing"}</span>
        <span>Trail: {wombatTrail.length} points</span>
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
