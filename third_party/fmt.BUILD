load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "fmt",
    hdrs = glob(["include/**/*.h"]),
    srcs = [
        "src/format.cc",
        "src/os.cc",
    ],
    includes = ["include"],
    copts = ["-Iexternal/com_github_fmtlib_fmt/src"],
    visibility = ["//visibility:public"],
)
