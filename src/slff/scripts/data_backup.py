#!/usr/bin/python3

import os
import subprocess

import schedule

import rospy


# =============================================================================
# -----------------------------------------------------------------------------
# =============================================================================


def cllbck_tim_1hz(event):
    if gdrive_backup_routine() == -1:
        rospy.signal_shutdown("signal shutdown")

# =============================================================================
# -----------------------------------------------------------------------------
# =============================================================================


def gdrive_backup_init():
    schedule.every(10).minutes.do(backup)
    schedule.run_all()

    return 0


def gdrive_backup_routine():
    schedule.run_pending()

    return 0

# =============================================================================
# -----------------------------------------------------------------------------
# =============================================================================


def backup():
    # Local path
    log_local = os.path.join(
        # /root
        os.getenv("HOME"),
        # /root/slff-data
        "slff-data",
        # /root/slff-data/log
        "log")

    # Remote path
    log_remote = os.path.join(
        # /Let it Flo
        "Let it Flo",
        # /Let it Flo/Kapuk - Gardu 9 - [20-09]
        "%s - Gardu %d - [%02d-%02d]" % (nama_gerbang, no_gardu, no_gerbang, no_gardu),
        # /Let it Flo/Kapuk - Gardu 9 - [20-09]/log
        "log")

    subprocess.Popen(["rclone", "copy", log_local, "Let it Flo - OneDrive:" + log_remote],
                     stdin=None, stdout=None, stderr=None, close_fds=True)

    return 0

# =============================================================================
# -----------------------------------------------------------------------------
# =============================================================================


if __name__ == "__main__":
    # Parameter
    global nama_gerbang
    global no_gardu
    global no_gerbang
    # Timer
    global tim_1hz

    rospy.init_node('gdrive_backup')

    # Parameter
    nama_gerbang = rospy.get_param('nama_gerbang', 'Undefined')
    no_gardu = rospy.get_param('no_gardu', 255)
    no_gerbang = rospy.get_param('no_gerbang', 255)
    # Timer
    tim_1hz = rospy.Timer(rospy.Duration(1), cllbck_tim_1hz)

    if gdrive_backup_init() == -1:
        rospy.signal_shutdown("signal shutdown")

    rospy.spin()
