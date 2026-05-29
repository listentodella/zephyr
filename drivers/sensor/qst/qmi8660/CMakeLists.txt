# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources(
    qmi8660.c
    qmi8660_bus.c
)

zephyr_library_sources_ifdef(CONFIG_SENSOR_ASYNC_API qmi8660_rtio.c)
zephyr_library_sources_ifdef(CONFIG_QMI8660_DECODER qmi8660_decoder.c)
zephyr_library_sources_ifdef(CONFIG_QMI8660_STREAM qmi8660_rtio_stream.c)
