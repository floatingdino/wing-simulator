-- @description WING: store scene
-- @about
--   Stores the simulator's current state as a library scene (/wingsim/scene/store) and,
--   optionally, adds a "WING:<name>" marker at the edit cursor so playback recalls it later
--   (with "WING - Recall scenes at markers" or the WingSim-*.ReaperOSC control surface).
--   An empty name overwrites the active scene (or appends "Scene N" when none is active).
--   Scene names: at most 64 bytes, no / \ : or commas (the dialog uses commas as separators).
--   Requires wing_osc.lua in the same folder. See docs/user/reaper-integration.md.

local dir = ({ reaper.get_action_context() })[2]:match("^(.*[/\\])")
local wing = dofile(dir .. "wing_osc.lua")

-- Default name: the WING marker/region at or before the edit cursor.
local function current_target()
  local cur = reaper.GetCursorPosition()
  local best, best_pos = "", -math.huge
  local i = 0
  while true do
    local ok, _, pos, _, name = reaper.EnumProjectMarkers3(0, i)
    if ok == 0 then break end
    local t = wing.marker_target(name)
    if t and pos <= cur + 1e-9 and pos >= best_pos and not t:match("^#") then
      best, best_pos = t, pos
    end
    i = i + 1
  end
  return best
end

local ok, csv = reaper.GetUserInputs("WING: store scene", 2,
  "Scene name (empty = active),Add marker at edit cursor (y/n),extrawidth=120",
  current_target() .. ",n")
if not ok then return end
local name, add = csv:match("^(.-),(.*)$")
name = (name or csv):match("^%s*(.-)%s*$")
add = (add or ""):match("^%s*(.-)%s*$"):lower()

if name:find("[/\\:]") then
  reaper.MB("Scene names cannot contain / \\ or :", "WING: store scene", 0)
  return
end

wing.store(name)
wing.log(name == "" and "store active scene" or ("store scene " .. string.format("%q", name)))

if (add == "y" or add == "yes") and name ~= "" then
  reaper.Undo_BeginBlock()
  reaper.AddProjectMarker2(0, false, reaper.GetCursorPosition(), 0, "WING:" .. name, -1, 0)
  reaper.Undo_EndBlock("WING: add scene marker " .. name, -1)
end
