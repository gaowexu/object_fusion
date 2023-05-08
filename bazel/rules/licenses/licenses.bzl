"""Rule for storing license information."""

LicenseInfo = provider(
    doc = "Provides information about a licenses",
    fields = ["licenses_names", "approved_on_ecu"],
)

def _licenses_impl(ctx):
    provider = LicenseInfo(
        licenses_names = ctx.attr.licenses_names,
        approved_on_ecu = ctx.attr.approved_on_ecu,
    )
    return [provider]

smart_licenses = rule(
    implementation = _licenses_impl,
    attrs = {
        "licenses_names": attr.string_list(),
        "approved_on_ecu": attr.bool(),
    },
)

def licenses(licenses_info, approved_for_usage_on_ecu = False):
    smart_licenses(
        name = "licenses",
        licenses_names = licenses_info,
        approved_on_ecu = approved_for_usage_on_ecu,
    )
