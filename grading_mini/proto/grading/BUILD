load("@rules_cc//cc:defs.bzl", "cc_proto_library")

proto_library(
    name = "scene_proto",
    srcs = ["scene.proto"],
    visibility = ["//visibility:public"],
)

cc_proto_library(
    name = "scene_cc_proto",
    deps = [":scene_proto"],
    visibility = ["//visibility:public"],
)

proto_library(
    name = "metric_input_proto",
    srcs = ["metric_input.proto"],
    deps = [":scene_proto"],
    visibility = ["//visibility:public"],
)

cc_proto_library(
    name = "metric_input_cc_proto",
    deps = [":metric_input_proto"],
    visibility = ["//visibility:public"],
)

proto_library(
    name = "metric_output_proto",
    srcs = ["metric_output.proto"],
    deps = ["@com_google_protobuf//:any_proto"],
    visibility = ["//visibility:public"],
)

cc_proto_library(
    name = "metric_output_cc_proto",
    deps = [":metric_output_proto"],
    visibility = ["//visibility:public"],
)

proto_library(
    name = "sim_log_proto",
    srcs = ["sim_log.proto"],
    deps = [":metric_input_proto"],
    visibility = ["//visibility:public"],
)

cc_proto_library(
    name = "sim_log_cc_proto",
    deps = [":sim_log_proto"],
    visibility = ["//visibility:public"],
)

proto_library(
    name = "grading_run_config_proto",
    srcs = ["grading_run_config.proto"],
    visibility = ["//visibility:public"],
)

cc_proto_library(
    name = "grading_run_config_cc_proto",
    deps = [":grading_run_config_proto"],
    visibility = ["//visibility:public"],
)
