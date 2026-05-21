load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("@hedron_compile_commands//:refresh_compile_commands.bzl", "refresh_compile_commands")

refresh_compile_commands(
    name = "refresh_compile_commands",
    # 可选：如果你想限定只为特定的 target 生成，可以取消下面这行的注释并填入 targets
    # targets = ["//my_package:my_target"],
)

cc_library(
    name = "json_utils",
    srcs = ["json_utils.cc"],
    hdrs = ["json_utils.h"],
    deps = ["@com_google_protobuf//:protobuf"],
)

cc_library(
    name = "input_format",
    srcs = ["input_format.cc"],
    hdrs = ["input_format.h"],
)

cc_library(
    name = "proto_io",
    srcs = ["proto_io.cc"],
    hdrs = ["proto_io.h"],
    deps = [
        ":input_format",
        ":json_utils",
        "//proto/sim:map_cc_proto",
        "//proto/sim:scenario_cc_proto",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "dynamic_source",
    srcs = ["dynamic_source.cc"],
    hdrs = ["dynamic_source.h"],
    deps = [
        ":input_format",
        ":json_utils",
        ":proto_io",
        "//proto/sim:runtime_cc_proto",
        "//proto/sim:scenario_cc_proto",
    ],
)

cc_library(
    name = "scenario_loader",
    srcs = ["scenario_loader.cc"],
    hdrs = ["scenario_loader.h"],
    deps = [
        ":dynamic_source",
        ":input_format",
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
        ":dynamic_source",
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
        "@com_google_protobuf//:protobuf",
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
