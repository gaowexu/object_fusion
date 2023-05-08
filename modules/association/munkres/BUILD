load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "munkres",
    srcs = [
        "munkres.cpp",
    ],
    hdrs = [
        "munkres.h",
    ],
    copts = ["-std=c++17"],
    features = [
        "treat_warnings_as_errors",
        "strict_clang_tidy_warnings",
        "strict_warnings",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//modules/types:object_fusion_common",
    ],
)
