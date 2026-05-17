load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

cc_library(
    name = "types",
    hdrs = ["types.h"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "json_utils",
    srcs = ["json_utils.cc"],
    hdrs = ["json_utils.h"],
    deps = [
        ":types",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "scenario_loader",
    srcs = ["scenario_loader.cc"],
    hdrs = ["scenario_loader.h"],
    deps = [
        ":json_utils",
        ":types",
    ],
)

cc_library(
    name = "geometry",
    srcs = ["geometry.cc"],
    hdrs = ["geometry.h"],
    deps = [":types"],
)

cc_library(
    name = "lane_graph",
    srcs = ["lane_graph.cc"],
    hdrs = ["lane_graph.h"],
    deps = [
        ":json_utils",
        ":planner",
        ":types",
    ],
)

cc_library(
    name = "planner",
    srcs = ["planner.cc"],
    hdrs = ["planner.h"],
    deps = [
        ":geometry",
        ":types",
    ],
)

cc_library(
    name = "world",
    srcs = ["world.cc"],
    hdrs = ["world.h"],
    deps = [
        ":geometry",
        ":planner",
        ":types",
    ],
)

cc_library(
    name = "grading_bridge",
    srcs = ["grading_bridge.cc"],
    hdrs = ["grading_bridge.h"],
    deps = [":types"],
)

cc_library(
    name = "sim_logger",
    srcs = ["sim_logger.cc"],
    hdrs = ["sim_logger.h"],
    deps = [
        ":types",
        "@com_github_gabime_spdlog//:spdlog",
    ],
)

cc_binary(
    name = "sim_runner",
    srcs = ["main.cc"],
    deps = [
        ":grading_bridge",
        ":lane_graph",
        ":planner",
        ":scenario_loader",
        ":sim_logger",
        ":types",
        ":world",
    ],
)
