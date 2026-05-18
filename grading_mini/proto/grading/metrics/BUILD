load("@rules_cc//cc:defs.bzl", "cc_proto_library")

proto_library(
    name = "example_metric_proto",
    srcs = ["example_metric.proto"],
    visibility = ["//visibility:public"],
)

cc_proto_library(
    name = "example_metric_cc_proto",
    deps = [":example_metric_proto"],
    visibility = ["//visibility:public"],
)

proto_library(
    name = "safety_metric_proto",
    srcs = ["safety_metric.proto"],
    visibility = ["//visibility:public"],
)

cc_proto_library(
    name = "safety_metric_cc_proto",
    deps = [":safety_metric_proto"],
    visibility = ["//visibility:public"],
)
