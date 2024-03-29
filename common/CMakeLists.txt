# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.

set(COMMON_SRC
        "${CMAKE_CURRENT_LIST_DIR}/rendering.h"
        "${CMAKE_CURRENT_LIST_DIR}/model-views.h"
        "${CMAKE_CURRENT_LIST_DIR}/model-views.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/notifications.h"
        "${CMAKE_CURRENT_LIST_DIR}/notifications.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/calibration-model.h"
        "${CMAKE_CURRENT_LIST_DIR}/calibration-model.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/viewer.h"
        "${CMAKE_CURRENT_LIST_DIR}/viewer.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/ux-window.h"
        "${CMAKE_CURRENT_LIST_DIR}/ux-window.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/ux-alignment.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/ux-alignment.h"
        "${CMAKE_CURRENT_LIST_DIR}/opengl3.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/opengl3.h"
        "${CMAKE_CURRENT_LIST_DIR}/rs-config.h"
        "${CMAKE_CURRENT_LIST_DIR}/rs-config.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/os.h"
        "${CMAKE_CURRENT_LIST_DIR}/os.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/fw-update-helper.h"
        "${CMAKE_CURRENT_LIST_DIR}/fw-update-helper.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/metadata-helper.h"
        "${CMAKE_CURRENT_LIST_DIR}/metadata-helper.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/output-model.h"
        "${CMAKE_CURRENT_LIST_DIR}/output-model.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/skybox.h"
        "${CMAKE_CURRENT_LIST_DIR}/skybox.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/measurement.h"
        "${CMAKE_CURRENT_LIST_DIR}/measurement.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/on-chip-calib.h"
        "${CMAKE_CURRENT_LIST_DIR}/on-chip-calib.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/decompress-huffman.h"
        "${CMAKE_CURRENT_LIST_DIR}/updates-model.h"
        "${CMAKE_CURRENT_LIST_DIR}/updates-model.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/al-utilities.h"
        "${CMAKE_CURRENT_LIST_DIR}/al-utilities.cpp"		
        )

    set(SW_UPDATE_FILES
        "${CMAKE_CURRENT_LIST_DIR}/sw-update/http-downloader.h"
        "${CMAKE_CURRENT_LIST_DIR}/sw-update/http-downloader.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/sw-update/dev-updates-profile.h"
        "${CMAKE_CURRENT_LIST_DIR}/sw-update/dev-updates-profile.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/sw-update/versions-db-manager.h"
        "${CMAKE_CURRENT_LIST_DIR}/sw-update/versions-db-manager.cpp"
        )

    set(REFLECTIVITY_FILES
        "${CMAKE_CURRENT_LIST_DIR}/reflectivity/reflectivity.h"
        "${CMAKE_CURRENT_LIST_DIR}/reflectivity/reflectivity.cpp"
        )

    set(UTILITIES_FILES
        "${CMAKE_CURRENT_LIST_DIR}/utilities/number/stabilized-value.h"
        "${CMAKE_CURRENT_LIST_DIR}/utilities/string/trim-newlines.h"
        "${CMAKE_CURRENT_LIST_DIR}/utilities/string/split.h"
        "${CMAKE_CURRENT_LIST_DIR}/utilities/imgui/wrap.h"
        "${CMAKE_CURRENT_LIST_DIR}/utilities/imgui/wrap.cpp"
        )

    set(COMMON_SRC
        ${COMMON_SRC}
        ${SW_UPDATE_FILES}
        ${REFLECTIVITY_FILES}
        ${UTILITIES_FILES}
        )

