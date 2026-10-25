-- wing_osc.lua: shared helpers for the "WING - ..." ReaScripts (Wing Simulator, T065).
--
-- REAPER's Lua has no sockets, so a message is encoded here (OSC 1.0, ,s ,i ,f arguments),
-- written to a temporary file and handed to the system `nc` (netcat) as one UDP datagram,
-- in the background so REAPER never waits. macOS and Linux only (Windows has no `nc`).
-- Each send starts a process: fine for scene changes, too heavy for continuous automation
-- (use the WingSim-Faders.ReaperOSC control surface for that; see docs/user/reaper-integration.md).
--
-- Settings are read from REAPER's ExtState section "WingSim" (keys "host", "port"), so they can
-- be changed without editing scripts:  reaper.SetExtState("WingSim", "port", "2223", true)

local M = {}

M.SECTION = "WingSim"
M.DEFAULT_HOST = "127.0.0.1"
M.DEFAULT_PORT = 2223
M.NC = "/usr/bin/nc"

function M.host()
  local h = reaper.GetExtState(M.SECTION, "host")
  if h == nil or h == "" then return M.DEFAULT_HOST end
  return h
end

function M.port()
  local p = tonumber(reaper.GetExtState(M.SECTION, "port"))
  if p == nil then return M.DEFAULT_PORT end
  return math.floor(p)
end

-- OSC string: bytes, NUL terminator, padded to a multiple of 4.
local function osc_str(s)
  local n = #s + 1
  local pad = (4 - n % 4) % 4
  return s .. string.rep("\0", 1 + pad)
end

-- Encode one OSC message. args: list of values; Lua strings -> ,s, integers -> ,i,
-- other numbers -> ,f. Use {f = x} to force a float.
function M.encode(addr, args)
  local tags, body = ",", {}
  for _, a in ipairs(args or {}) do
    if type(a) == "string" then
      tags = tags .. "s"
      body[#body + 1] = osc_str(a)
    elseif type(a) == "table" and a.f ~= nil then
      tags = tags .. "f"
      body[#body + 1] = string.pack(">f", a.f)
    elseif math.type(a) == "integer" then
      tags = tags .. "i"
      body[#body + 1] = string.pack(">i4", a)
    elseif type(a) == "number" then
      tags = tags .. "f"
      body[#body + 1] = string.pack(">f", a)
    else
      error("wing_osc: unsupported OSC argument " .. tostring(a))
    end
  end
  return osc_str(addr) .. osc_str(tags) .. table.concat(body)
end

local function shell_quote(s)
  return "'" .. string.gsub(s, "'", "'\\''") .. "'"
end

-- Send one message to the simulator (non-blocking). Returns true when the send was started.
function M.send(addr, args)
  local path = os.tmpname()
  local f = io.open(path, "wb")
  if not f then
    reaper.ShowConsoleMsg("WING: cannot write " .. path .. "\n")
    return false
  end
  f:write(M.encode(addr, args))
  f:close()
  local cmd = string.format("(%s -u -w 1 %s %d < %s; rm -f %s) >/dev/null 2>&1 &",
    M.NC, shell_quote(M.host()), M.port(), shell_quote(path), shell_quote(path))
  os.execute(cmd)
  return true
end

-- The scene named by a "WING:<scene or #N>" marker/region name, or nil. Same rule as the
-- daemon's /wingsim/marker (prefix case-insensitive, surrounding blanks ignored).
function M.marker_target(name)
  if type(name) ~= "string" then return nil end
  local n = name:match("^%s*(.-)%s*$")
  if n:sub(1, 5):upper() ~= "WING:" then return nil end
  local t = n:sub(6):match("^%s*(.-)%s*$")
  if t == "" then return nil end
  return t
end

function M.recall(scene)
  if math.type(scene) == "integer" then
    return M.send("/wingsim/scene/recall", { scene })
  end
  return M.send("/wingsim/scene/recall", { tostring(scene) })
end

function M.store(name)
  if name == nil or name == "" then
    return M.send("/wingsim/scene/store", {})
  end
  return M.send("/wingsim/scene/store", { name })
end

function M.log(msg)
  reaper.ShowConsoleMsg("WING: " .. msg .. "\n")
end

-- Load this module from a script in the same folder:
--   local dir = ({reaper.get_action_context()})[2]:match("^(.*[/\\])")
--   local wing = dofile(dir .. "wing_osc.lua")
return M
