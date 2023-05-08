load("@//third_party:remotes.bzl", "THIRD_PARTY_RESOURCES")
load("@//bazel/rules:artifactory.bzl", "artifactory_archive")

def load_eigen():
    if "eigen" not in native.existing_rules():
        artifactory_archive(
            name = "eigen",
            path = "eigen-3.4.0.tar.gz",
            repo = THIRD_PARTY_RESOURCES,
            sha256 = "8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72",
            strip_prefix = "eigen-3.4.0",
            build_file = "//third_party/eigen:eigen.BUILD",
        )
