.PHONY: dm-sync dm-test gate
dm-sync:
	cd packages/fastgripper-dm && uv sync --extra dev
dm-test: dm-sync
	cd packages/fastgripper-dm && uv run pytest -q
gate:
	@! grep -rnE 'usbmodem[0-9]{7,}' packages bench docs 2>/dev/null || (echo "serial numbers / bench device paths found" && exit 1)
