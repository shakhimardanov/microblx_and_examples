#!/usr/bin/luajit

local ffi = require("ffi")
local ubx = require "lua/ubx"
local ubx_utils = require("lua/ubx_utils")
local ts = tostring

-- prog starts here.
ni=ubx.node_create("testnode")

ubx.load_module(ni, "std_types/stdtypes/stdtypes.so")
ubx.load_module(ni, "std_blocks/webif/webif.so")
ubx.load_module(ni, "std_blocks/cppdemo/cppdemo.so")
ubx.ffi_load_types(ni)

print("creating instance of 'webif/webif'")
webif1=ubx.block_create(ni, "webif/webif", "webif1", { port="8888" })

print("creating instance of 'cppdemo/cppdemo'")
cppdemo1=ubx.block_create(ni, "cppdemo/cppdemo", "cppdemo1")

print("running webif init", ubx.block_init(ni, webif1))
print("running webif start", ubx.block_start(ni, webif1))

io.read()

print("running cppdemo1 unload", ubx.block_unload(ni, "cppdemo1"))
print("running webif1 unload", ubx.block_unload(ni, "webif1"))

ubx.unload_modules(ni)


