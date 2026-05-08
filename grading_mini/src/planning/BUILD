load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "simple_planner",
    srcs = ["simple_planner.cc"],
    hdrs = ["simple_planner.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//proto/grading:metric_input_cc_proto",
    ],
)
