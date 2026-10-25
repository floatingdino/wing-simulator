-- @description WING: next scene
-- @about Recalls the library scene after the active one (/wingsim/scene/next). Requires wing_osc.lua.

local dir = ({ reaper.get_action_context() })[2]:match("^(.*[/\\])")
local wing = dofile(dir .. "wing_osc.lua")
wing.send("/wingsim/scene/next", {})
