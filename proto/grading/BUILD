load("@rules_cc//cc:defs.bzl", "cc_proto_library")

proto_library(
    name = "metric_input_proto",
    srcs = ["metric_input.proto"],
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
