load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "agent_associator",
    srcs = [
        "agent_associator.cpp",
    ],
    hdrs = [
        "agent_associator.h",
    ],
    features = [
        "treat_warnings_as_errors",
        "strict_clang_tidy_warnings",
        "strict_warnings",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//modules/association/munkres",
        "//modules/types:object_fusion_common",
        "//third_party/eigen",
    ],
)
