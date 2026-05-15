load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "spdlog",
    hdrs = glob(["include/**/*.h"]),
    includes = ["include"],
    defines = [
        "SPDLOG_FMT_EXTERNAL",
        "SPDLOG_COMPILED_LIB",
    ],
    srcs = glob(["src/*.cpp"]),
    deps = ["@com_github_fmtlib_fmt//:fmt"],
    visibility = ["//visibility:public"],
)
