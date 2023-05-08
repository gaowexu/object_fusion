"""Rules to provide a debian package hosted on artifactory as an external project."""

load("//bazel/rules:artifactory.bzl", "artifactory_file")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "workspace_and_buildfile")

def artifactory_deb(name, repo, path, sha256, strip_prefix = "", build_file = None, build_file_content = None):
    """Macro to provide a debian package hosted on artifactory as an external project.

    Args:
        name: the name for the workspace created for the debian package
        repo: The url to the artifactory repository.
              E.g. TBD.
        path: The url path to the file inside the artifactory repository. E.g. /test/test-1.0.txt.
        sha256: A sha256 hash of the file to be downloaded. It can be obtained from the artifactory api or via the web
                interface.
        strip_prefix: A directory prefix to strip from the extracted files.
        build_file: The file to use as the BUILD file for this repository. This attribute is an absolute label
                    (use '@//' for the main repo). The file does not need to be named BUILD, but can be (something like
                    BUILD.new-repo-name may work well for distinguishing it from the repository's actual BUILD files.
                    Either build_file or build_file_content must be specified.
        build_file_content: The content for the BUILD file for this repository.
                            Either build_file or build_file_content must be specified.

    To use prebuilt debian packages via bazel, instead of installing them with a package manager, declare them with this
    rule. This will pull them from artifactory and extract them.
    """
    artifactory_deb_group(
        name = name,
        repo = repo,
        package_group = {path: sha256},
        strip_prefix = strip_prefix,
        build_file = build_file,
        build_file_content = build_file_content,
    )

def artifactory_deb_group(name, repo, package_group, strip_prefix = "", build_file = None, build_file_content = None):
    """Macro to provide multiple debian packages hosted on artifactory as a single external project.

    Args:
        name: the name for the workspace created for the debian package
        repo: The url to the artifactory repository.
        package_group: A dict of path/sha256 key-value pairs, where path points to the file inside the artifactory
                       repository and sha256 is the file's sha256 hash.
        strip_prefix: A directory prefix to strip from the extracted files.
        build_file: The file to use as the BUILD file for this repository. This attribute is an absolute label
                    (use '@//' for the main repo). The file does not need to be named BUILD, but can be (something like
                    BUILD.new-repo-name may work well for distinguishing it from the repository's actual BUILD files.
                    Either build_file or build_file_content must be specified.
        build_file_content: The content for the BUILD file for this repository.
                            Either build_file or build_file_content must be specified.
    """
    file_labels = []
    for path, sha256 in package_group.items():
        file_label = "{}_{}".format(name, sha256)
        file_labels.append(file_label)
        artifactory_file(name = file_label, repo = repo, path = path, sha256 = sha256, downloaded_file_path = "downloaded")

    file_targets = ["@{}//file:downloaded".format(file_label) for file_label in file_labels]

    _debian_archives(
        name = name,
        files = file_targets,
        strip_prefix = strip_prefix,
        build_file = build_file,
        build_file_content = build_file_content,
    )

def _debian_archives_impl(repo_ctx):
    for file_label in repo_ctx.attr.files:
        repo_ctx.report_progress("Extracting {}".format(file_label))
        deb_file = repo_ctx.path(file_label)
        _extract_debian_package(repo_ctx, deb_file)

    workspace_and_buildfile(repo_ctx)

_debian_archives = repository_rule(
    implementation = _debian_archives_impl,
    local = False,
    attrs = {
        "files": attr.label_list(mandatory = True, allow_empty = False, allow_files = True),
        "strip_prefix": attr.string(),
        "build_file": attr.label(),
        "build_file_content": attr.string(),
        "workspace_file": attr.label(),
        "workspace_file_content": attr.string(),
        "_ar_tool": attr.string(default = "/usr/bin/ar"),
    },
)

def _extract_debian_package(repo_ctx, deb_file):
    file_list = _list_files_in_archive(repo_ctx, deb_file)
    data_archive_filename = _find_data_archive(repo_ctx, file_list)
    res = repo_ctx.execute([repo_ctx.attr._ar_tool, "-x", deb_file, data_archive_filename])
    if res.return_code != 0:
        fail("Running {} failed with stderr:\n{}".format(repo_ctx.attr._ar_tool, res.stderr))
    repo_ctx.extract(
        data_archive_filename,
        stripPrefix = repo_ctx.attr.strip_prefix,
    )
    repo_ctx.delete(data_archive_filename)

def _list_files_in_archive(repo_ctx, deb_file):
    result = repo_ctx.execute([repo_ctx.attr._ar_tool, "-t", deb_file])
    if result.return_code != 0:
        fail("Running {} failed with stderr:\n{}".format(repo_ctx.attr._ar_tool, result.stderr))
    file_list = [item for item in result.stdout.split("\n") if item]
    return file_list

def _find_data_archive(repo_ctx, file_list):
    data_archives = [item for item in file_list if item.startswith("data.tar")]
    if not data_archives:
        fail("Debian package {} doesn't contain a data archive.".format(repo_ctx.name))
    if len(data_archives) > 1:
        fail("Debian package {} contains multiple data archives.".format(repo_ctx.name))
    return data_archives[0]
