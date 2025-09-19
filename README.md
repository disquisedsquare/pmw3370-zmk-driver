# zmk-pmw3370-driver

Standalone PMW3370 driver module for Zephyr/ZMK.

## How to include in your ZMK build

Add this repository to your top-level `west.yml` (or include it in your workspace).
Example `west.yml` entry in your zmk-config repo:

manifest:
remotes:
- name: zmkfirmware
url-base: https://github.com/zmkfirmware

- name: pmw3370
url-base: https://github.com/your-gh-account

projects:
- name: zmk
remote: zmkfirmware
revision: main
import: app/west.yml
- name: zmk-pmw3370-driver
remote: pmw3370
revision: main
path: modules/zmk-pmw3370-driver
self:
path: config
