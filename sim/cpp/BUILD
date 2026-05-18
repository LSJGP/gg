load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

cc_library(
    name = "json_utils",
    srcs = ["json_utils.cc"],
    hdrs = ["json_utils.h"],
    deps = ["@com_google_protobuf//:protobuf"],
)

cc_library(
    name = "proto_io",
    srcs = ["proto_io.cc"],
    hdrs = ["proto_io.h"],
    deps = [
        ":json_utils",
        "//proto/sim:map_cc_proto",
        "//proto/sim:scenario_cc_proto",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "scenario_loader",
    srcs = ["scenario_loader.cc"],
    hdrs = ["scenario_loader.h"],
    deps = [
        ":proto_io",
        "//proto/sim:map_cc_proto",
        "//proto/sim:scenario_cc_proto",
    ],
)

cc_library(
    name = "geometry",
    srcs = ["geometry.cc"],
    hdrs = ["geometry.h"],
)

cc_library(
    name = "lane_graph",
    srcs = ["lane_graph.cc"],
    hdrs = ["lane_graph.h"],
    deps = [
        ":proto_io",
        "//proto/sim:map_cc_proto",
        "//proto/sim:runtime_cc_proto",
        "//proto/sim:scenario_cc_proto",
    ],
)

cc_library(
    name = "map_query",
    srcs = ["map_query.cc"],
    hdrs = ["map_query.h"],
    deps = [
        ":lane_graph",
        "//proto/sim:runtime_cc_proto",
    ],
)

cc_library(
    name = "trajectory_tracker",
    srcs = ["trajectory_tracker.cc"],
    hdrs = ["trajectory_tracker.h"],
    deps = [
        "//proto/sim:runtime_cc_proto",
    ],
)

cc_library(
    name = "planner",
    srcs = ["planner.cc"],
    hdrs = ["planner.h"],
    deps = [
        ":geometry",
        ":trajectory_tracker",
        "//proto/sim:runtime_cc_proto",
    ],
)

cc_library(
    name = "world",
    srcs = ["world.cc"],
    hdrs = ["world.h"],
    deps = [
        ":lane_graph",
        ":map_query",
        ":planner",
        ":trajectory_tracker",
        "//proto/sim:runtime_cc_proto",
        "//proto/sim:scenario_cc_proto",
    ],
)

cc_library(
    name = "grading_convert",
    srcs = ["grading_convert.cc"],
    hdrs = ["grading_convert.h"],
    deps = [
        "//proto/sim:map_cc_proto",
        "//proto/sim:runtime_cc_proto",
        "@grading_mini//proto/grading:metric_input_cc_proto",
    ],
)

cc_library(
    name = "grading_bridge",
    srcs = ["grading_bridge.cc"],
    hdrs = ["grading_bridge.h"],
    deps = [
        ":grading_convert",
        "//proto/sim:map_cc_proto",
        "//proto/sim:runtime_cc_proto",
        "@com_google_protobuf//:protobuf",
        "@grading_mini//proto/grading:metric_input_cc_proto",
        "@grading_mini//proto/grading:sim_log_cc_proto",
    ],
)

cc_library(
    name = "sim_logger",
    srcs = ["sim_logger.cc"],
    hdrs = ["sim_logger.h"],
    deps = [
        "//proto/sim:runtime_cc_proto",
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
        ":world",
        "//proto/sim:runtime_cc_proto",
    ],
)
