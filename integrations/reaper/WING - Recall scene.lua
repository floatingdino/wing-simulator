-- @description WING: recall scene
-- @about
--   Asks for a scene name (or "#N", 0-based library position) and recalls it on the simulator
--   (/wingsim/scene/recall). The default is the WING marker/region at or before the edit cursor.
--   Requires wing_osc.lua in the same folder. See docs/user/reaper-integration.md.

local dir = ({ reaper.get_action_context() })[2]:match("^(.*[/\\])")
local wing = dofile(dir .. "wing_osc.lua")

local cur = reaper.GetCursorPosition()
local default, best_pos = "", -math.huge
local i = 0
while true do
  local ok, _, pos, _, name = reaper.EnumProjectMarkers3(0, i)
  if ok == 0 then break end
  local t = wing.marker_target(name)
  if t and pos <= cur + 1e-9 and pos >= best_pos then default, best_pos = t, pos end
  i = i + 1
end

local ok, name = reaper.GetUserInputs("WING: recall scene", 1,
  "Scene name or #N,extrawidth=120", default)
if not ok then return end
name = name:match("^%s*(.-)%s*$")
if name == "" then return end
wing.recall(name)
wing.log("recall " .. string.format("%q", name))
