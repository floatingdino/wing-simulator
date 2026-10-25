-- @description WING: recall scene at marker (follow playback)
-- @about
--   Background script for the Wing Simulator (T065). While REAPER plays, every marker or region
--   named "WING:<scene name>" or "WING:#<n>" (0-based library position) that the play cursor
--   crosses recalls that scene on the simulator over OSC (/wingsim/scene/recall).
--   On play start, on a loop wrap and after a seek it "chases": the last WING marker/region start
--   at or before the play position is recalled, so the console always matches the timeline.
--   Run it once to start, run it again to stop (toolbar buttons show the on/off state).
--   Requires wing_osc.lua in the same folder. See docs/user/reaper-integration.md.

local dir = ({ reaper.get_action_context() })[2]:match("^(.*[/\\])")
local wing = dofile(dir .. "wing_osc.lua")

-- Settings
local CHASE = true      -- recall the current scene on play start / loop / seek
local MAX_STEP = 1.0    -- seconds; a bigger forward jump between ticks counts as a seek

local _, _, section, cmd = reaper.get_action_context()
if reaper.set_action_options then reaper.set_action_options(1) end -- re-run terminates

local marks = {}        -- sorted { pos = seconds, target = scene } of WING markers/region starts
local marks_state = -1
local was_playing = false
local prev_pos = 0

local function refresh_marks()
  local state = reaper.GetProjectStateChangeCount(0)
  if state == marks_state then return end
  marks_state = state
  marks = {}
  local i = 0
  while true do
    local ok, _, pos, _, name = reaper.EnumProjectMarkers3(0, i)
    if ok == 0 then break end
    local t = wing.marker_target(name)
    if t then marks[#marks + 1] = { pos = pos, target = t } end
    i = i + 1
  end
  table.sort(marks, function(a, b) return a.pos < b.pos end)
end

-- Last WING mark with lo < pos <= hi (lo = nil: any pos <= hi).
local function last_mark(lo, hi)
  local found = nil
  for _, m in ipairs(marks) do
    if m.pos > hi + 1e-9 then break end
    if lo == nil or m.pos > lo + 1e-9 then found = m end
  end
  return found
end

local function recall(m, why)
  wing.recall(m.target) -- names and "#N" both go as text; the daemon resolves them
  wing.log(string.format("%s at %.3f s -> recall %q", why, m.pos, m.target))
end

local function tick()
  local playing = (reaper.GetPlayState() & 1) == 1
  if playing then
    refresh_marks()
    local pos = reaper.GetPlayPosition() -- audible position (latency compensated)
    if not was_playing or pos < prev_pos or pos - prev_pos > MAX_STEP then
      if CHASE then
        local m = last_mark(nil, pos)
        if m then recall(m, was_playing and "seek/loop" or "play") end
      end
    else
      local m = last_mark(prev_pos, pos)
      if m then recall(m, "marker") end
    end
    prev_pos = pos
  end
  was_playing = playing
  reaper.defer(tick)
end

local function set_toggle(on)
  reaper.SetToggleCommandState(section, cmd, on and 1 or 0)
  reaper.RefreshToolbar2(section, cmd)
end

set_toggle(true)
reaper.atexit(function()
  set_toggle(false)
  wing.log("marker follower stopped")
end)
wing.log(string.format("marker follower running (sending to %s:%d)", wing.host(), wing.port()))
tick()
