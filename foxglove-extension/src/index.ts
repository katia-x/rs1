import { ExtensionContext } from "@foxglove/extension";

import { initRegionSelectionPanel } from "./RegionSelectionPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "rs1-region-selection",
    initPanel: initRegionSelectionPanel,
  });
}
