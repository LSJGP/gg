load("@rules_cc//cc:defs.bzl", "cc_proto_library")

proto_library(
    name = "common_proto",
    srcs = ["common.proto"],
    visibility = ["//visibility:public"],
)

cc_proto_library(
    name = "common_cc_proto",
    deps = [":common_proto"],
    visibility = ["//visibility:public"],
)

proto_library(
    name = "scenario_proto",
    srcs = ["scenario.proto"],
    deps = [":common_proto"],
    visibility = ["//visibility:public"],
)

cc_proto_library(
    name = "scenario_cc_proto",
    deps = [":scenario_proto"],
    visibility = ["//visibility:public"],
)

proto_library(
    name = "map_proto",
    srcs = ["map.proto"],
    deps = [":common_proto"],
    visibility = ["//visibility:public"],
)

cc_proto_library(
    name = "map_cc_proto",
    deps = [":map_proto"],
    visibility = ["//visibility:public"],
)

proto_library(
    name = "runtime_proto",
    srcs = ["runtime.proto"],
    deps = [":common_proto"],
    visibility = ["//visibility:public"],
)

cc_proto_library(
    name = "runtime_cc_proto",
    deps = [":runtime_proto"],
    visibility = ["//visibility:public"],
)

proto_library(
    name = "sim_proto",
    srcs = ["sim.proto"],
    deps = [
        ":common_proto",     # 依赖基础的 proto_library
        ":map_proto",
        ":runtime_proto",
        ":scenario_proto",
    ],
)

cc_proto_library(
    name = "sim_cc_proto",
    deps = [":sim_proto"],   # <--- 这里只能写 proto_library
    visibility = ["//visibility:public"],
)
