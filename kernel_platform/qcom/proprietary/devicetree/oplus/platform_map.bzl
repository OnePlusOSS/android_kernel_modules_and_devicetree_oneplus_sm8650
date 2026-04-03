_platform_map = {
    "autoghgvm": {
        "dtb_list": [

        ],
        "dtbo_list": [

        ],
    },
    "autogvm": {
        "dtb_list": [

        ],
        "dtbo_list": [

        ],
    },
    "gen3auto": {
        "dtb_list": [

        ],
        "dtbo_list": [

        ],
    },
    "gen4auto": {
        "dtb_list": [

        ],
        "dtbo_list": [

        ],
    },
    "sdmsteppeauto": {
        "dtb_list": [

        ],
        "dtbo_list": [

        ],
    },
    "kalama": {
        "dtb_list": [

        ],
        "dtbo_list": [

        ],
    },
    "kalama-tuivm": {
        "dtb_list": [

        ],
    },
    "kalama-oemvm": {
        "dtb_list": [

        ],
    },
    "pineapple": {
        "dtb_list": [
            {"name": "pineapple.dtb"},
            {"name": "pineapple-v2.dtb"},
            {
                "name": "pineapplep.dtb",
                "apq": True,
            },
            {
                "name": "pineapplep-v2.dtb",
                "apq": True,
            },
        ],
        "dtbo_list": [
            {"name": "waffle-22825-pineapple-overlay.dtbo"},
            {"name": "waffle-22825-pineapple-overlay-EVB.dtbo"},
            {"name": "waffle-22825-pineapple-overlay-EVT1.dtbo"},
            {"name": "waffle-22877-pineapple-overlay.dtbo"},
            {"name": "waffle-22877-pineapple-overlay-EVB.dtbo"},
            {"name": "waffle-22877-pineapple-overlay-EVT1.dtbo"},
            {"name": "corvette-23814-pineapple-overlay.dtbo"},
            {"name": "caihong-23926-pineapple-overlay.dtbo"},
            {"name": "caihong-23976-pineapple-overlay.dtbo"},
            {"name": "caihong-23926-pineapple-overlay-T0.dtbo"},
            {"name": "caihong-23976-pineapple-overlay-T0.dtbo"},
            {"name": "giulia-23851-pineapple-overlay-T0.dtbo"},
            {"name": "giulia-23851-pineapple-overlay.dtbo"},
            {"name": "giulia-23867-pineapple-overlay-T0.dtbo"},
            {"name": "giulia-23867-pineapple-overlay-T0-EU.dtbo"},
            {"name": "giulia-23867-pineapple-overlay.dtbo"},
        ],
        "binary_compatible_with": ["cliffs", "volcano"],
    },
    "niobe": {
        "dtb_list": [

        ],
        "dtbo_list": [

        ],
    },
    "cliffs": {
        "dtb_list": [
            {"name": "cliffs.dtb"},
            {"name": "cliffs7.dtb"},
        ],
        "dtbo_list": [
            {"name": "audi-23803-cliffs-overlay.dtbo"},
            {"name": "audi-23865-cliffs-overlay.dtbo"},
            {"name": "avalon-24211-cliffs-overlay.dtbo"},
            {"name": "lexus-24881-cliffs-overlay.dtbo"},
            {"name": "lexus-24882-cliffs-overlay.dtbo"},
            {"name": "lexus-24882-EU-cliffs-overlay.dtbo"},
        ],
    },
    "volcano": {
        "dtb_list": [
            {"name": "volcano.dtb"},
            {"name": "volcano6.dtb"},
        ],
        "dtbo_list": [
            {"name": "prado-25811-volcano-overlay.dtbo"},
        ],
    },
    "pineapple-tuivm": {
        "dtb_list": [

        ],
    },
    "pineapple-oemvm": {
        "dtb_list": [

        ],
    },
    "blair": {
        "dtb_list": [

        ],
        "dtbo_list": [

        ],
    },
    "pitti": {
        "dtb_list": [

        ],
        "dtbo_list": [

        ],
    },
    "niobe-tuivm": {
        "dtb_list": [
            {"name": "niobe-vm-idp.dtb"},
            {"name": "niobep-vm-idp.dtb"},
        ],
    },
    "anorak": {
        "dtb_list": [
            {"name": "anorak.dtb"},
        ],
    },
}

def _get_dtb_lists(target, dt_overlay_supported):
    if not target in _platform_map:
        fail("{} not in device tree platform map!".format(target))

    ret = {
        "dtb_list": [],
        "dtbo_list": [],
    }

    for dtb_node in [target] + _platform_map[target].get("binary_compatible_with", []):
        ret["dtb_list"].extend(_platform_map[dtb_node].get("dtb_list", []))
        if dt_overlay_supported:
            ret["dtbo_list"].extend(_platform_map[dtb_node].get("dtbo_list", []))
        else:
            # Translate the dtbo list into dtbs we can append to main dtb_list
            for dtb in _platform_map[dtb_node].get("dtb_list", []):
                dtb_base = dtb["name"].replace(".dtb", "")
                for dtbo in _platform_map[dtb_node].get("dtbo_list", []):
                    if not dtbo.get("apq", True) and dtb.get("apq", False):
                        continue

                    dtbo_base = dtbo["name"].replace(".dtbo", "")
                    ret["dtb_list"].append({"name": "{}-{}.dtb".format(dtb_base, dtbo_base)})

    return ret

def get_dtb_list(target, dt_overlay_supported = True):
    return [dtb["name"] for dtb in _get_dtb_lists(target, dt_overlay_supported).get("dtb_list", [])]

def get_dtbo_list(target, dt_overlay_supported = True):
    return [dtb["name"] for dtb in _get_dtb_lists(target, dt_overlay_supported).get("dtbo_list", [])]
