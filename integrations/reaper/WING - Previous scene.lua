-- @description WING: previous scene
-- @about Recalls the library scene before the active one (/wingsim/scene/prev). Requires wing_osc.lua.

local dir = ({ reaper.get_action_context() })[2]:match("^(.*[/\\])")
local wing = dofile(dir .. "wing_osc.lua")
wing.send("/wingsim/scene/prev", {})
