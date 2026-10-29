import { ExtensionContext } from "@foxglove/extension";

import { initRegionSelectionPanel } from "./RegionSelectionPanel";
import { initOperatorPanel } from "./OperatorPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "rs1-region-selection",
    initPanel: initRegionSelectionPanel,
  });
  extensionContext.registerPanel({
    name: "rs1-wombat-operator",
    initPanel: initOperatorPanel,
  });
}
