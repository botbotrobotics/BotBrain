# utils.py
#!/usr/bin/env python
# -*- coding: UTF-8 -*-
# Copyright (C) 2020, Raffaello Bonghi <raffaello@rnext.it>
# All rights reserved
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
# BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rclpy
from copy import deepcopy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


def size_min(num, divider=1.0, n=0, start=''):
    if num >= divider * 1000.0:
        n += 1
        divider *= 1000.0
        return size_min(num, divider, n, start)
    else:
        vect = ['', 'K', 'M', 'G', 'T']
        idx = vect.index(start)
        return round(num / divider, 1), divider, vect[n + idx]


def strfdelta(tdelta, fmt):
    """ Print delta time
        - https://stackoverflow.com/questions/8906926/formatting-python-timedelta-objects
    """
    d = {"days": tdelta.days}
    d["hours"], rem = divmod(tdelta.seconds, 3600)
    d["minutes"], d["seconds"] = divmod(rem, 60)
    return fmt.format(**d)


def other_status(hardware, jetson, version):
    """
    Generic information about jetson_clock and nvpmodel
    """
    values = []
    nvpmodel = jetson.nvpmodel
    text = ""
    if nvpmodel is not None:
        nvp_name = nvpmodel.name.replace('MODE_', '').replace('_', ' ')
        values += [KeyValue(key="NV Power-ID", value=str(nvpmodel.id)),
                   KeyValue(key="NV Power-Mode", value=str(nvp_name))]
        text += "NV Power[{id}] {name}".format(id=nvpmodel.id, name=nvp_name)
    jc = jetson.jetson_clocks
    if jetson.jetson_clocks is not None:
        if jetson.jetson_clocks.status in ["running", "inactive"]:
            level = DiagnosticStatus.OK
        elif "ing" in jc.status:
            level = DiagnosticStatus.WARN
        else:
            level = DiagnosticStatus.ERROR
        # Show if JetsonClock is enabled or not
        values += [KeyValue(key="jetson_clocks", value=str(jc.status))]
        values += [KeyValue(key="jetson_clocks on boot", value=str(jc.boot))]
        text += " - JC {status}".format(status=jc.status)
    # Uptime
    uptime_string = strfdelta(
        jetson.uptime, "{days} days {hours}:{minutes}:{seconds}")
    values += [KeyValue(key="Up Time", value=str(uptime_string))]
    # Jtop version
    values += [KeyValue(key="interval", value=str(jetson.interval))]
    values += [KeyValue(key="jtop", value=str(version))]
    # Make board diagnostic status
    status = DiagnosticStatus(
        level=level,
        name='jetson_stats board status',
        message=text,
        hardware_id=hardware,
        values=values)
    return status


def board_status(hardware, board, dgtype):
    """
    Board information and libraries installed
    """
    values = []
    for key, value in board['hardware'].items():
        values += [KeyValue(key=key, value=str(value))]
    for key, value in board['libraries'].items():
        values += [KeyValue(key="lib " + key, value=str(value))]
    # Make board diagnostic status
    d_board = DiagnosticStatus(
        name='jetson_stats {type} config'.format(type=dgtype),
        message='Jetpack {jetpack}'.format(
            jetpack=board['hardware']['Jetpack']),
        hardware_id=hardware,
        values=values)
    return d_board


def disk_status(hardware, disk, dgtype):
    """
    Status disk

    Fields:
    * **total** - Total disk space in GB
    * **available** - Space available in GB
    * **used** - Disk space used in GB
    * **available_no_root**
    """
    value = int(float(disk['used']) / float(disk['total']) * 100.0)
    if value >= 90:
        level = DiagnosticStatus.ERROR
    elif value >= 70:
        level = DiagnosticStatus.WARN
    else:
        level = DiagnosticStatus.OK
    # Make board diagnostic status
    d_board = DiagnosticStatus(
        level=level,
        name='jetson_stats {type} disk'.format(type=dgtype),
        message="{0:2.1f}GB/{1:2.1f}GB".format(disk['used'], disk['total']),
        hardware_id=hardware,
        values=[
            KeyValue(key="Used", value=str(disk['used'])),
            KeyValue(key="Total", value=str(disk['total'])),
            KeyValue(key="Unit", value="GB")])
    return d_board


def cpu_status(hardware, name, cpu):
    """
    Decode a cpu stats

    Fields:
    ========== ================= =======================================
    Name       Type              Description
    ========== ================= =======================================
    online     Bool              Status core
    governor   str               Type of governor running on the core
    freq       dict              Frequency of the core 
    info_freq  dict              Frequency of the core
    idle_state dict              All Idle state running
    user       float             User percentage utilization
    nice       float             Nice percentage utilization
    system     float             System percentage utilization
    idle       float             Idle percentage
    model      str               Model core running
    ========== ================= =======================================
    """

    message = 'OFF'
    values = []
    if cpu:
        if 'idle' in cpu:
            # read value, don't use user + system as there are other usages see: https://unix.stackexchange.com/a/449868/395756
            val = 100 - cpu['idle']
            message = '{val}%'.format(val=val)
            # Make Diagnostic Status message with cpu info
            values = [
                KeyValue(key="Val", value=str(val)),
                KeyValue(key="Freq", value=str(cpu['freq']['cur'])),
                KeyValue(key="Unit", value="khz")]

        if 'governor' in cpu and cpu['governor']:
            values += [KeyValue(key="Governor",
                                value=str(cpu['governor']))]

        if 'model' in cpu and cpu['model']:
            values += [KeyValue(key="Model", value=str(cpu['model']))]

    # Make Diagnostic message
    d_cpu = DiagnosticStatus(
        name='jetson_stats cpu {name}'.format(name=name),
        message=message,
        hardware_id=hardware,
        values=values)
    return d_cpu


def gpu_status(hardware, name, gpu):
    """
    Decode and build a diagnostic status message
    Fields:
    ============= =================== ====================================================
    Name          Type                Description
    ============= =================== ====================================================
    type          str                 Type of GPU (integrated, discrete)
    status        dict                Status of GPU
    freq          dict                Frequency GPU
    power_control dict                *(Optional)* Type of power control
    ============= =================== ====================================================
    """
    raw = gpu.get('freq', 0)
    cur_khz = raw.get('cur', raw) if isinstance(raw, dict) else raw
    d_gpu = DiagnosticStatus(
        name=f'jetson_stats gpu {name}',
        message=f"{gpu['status']['load']}%",
        hardware_id=hardware,
        values=[KeyValue(key='Val', value=str(gpu['status']['load'])),
                KeyValue(key='Freq', value=str(cur_khz)),
                KeyValue(key='Unit', value="khz")])
    return d_gpu


def fan_status(hardware, name, fan):
    """
    Fan speed and type of control

    Fields:
    ============= =================== ====================================================
    Name          Type                Description
    ============= =================== ====================================================
    speed         list                List of speed between [0, 100]
    rpm           list                *(Optional)* List of RPM for each fan
    profile       str                 Fan Profile, read :py:func:~jtop.core.fan.Fan.all_profiles()
    governor      str                 (Jetson with JP5+) Governor fan
    control       str                 (Jetson with JP5+) Type of controller
    ============= =================== ====================================================
    """
    # Make fan diagnostic status
    d_fan = DiagnosticStatus(
        name='jetson_stats {name} fan'.format(name=name),
        message='speed={speed}%'.format(speed=fan['speed']),
        hardware_id=hardware,
        values=[
            KeyValue(key='Mode', value=str(fan['profile'])),
            KeyValue(key="Speed", value=str(fan['speed'])),
            KeyValue(key="Control", value=str(fan['control'])),
        ])
    return d_fan


def ram_status(hardware, ram, dgtype):
    """
    Make a RAM diagnostic status message

    Fields:
    ========== =================== ====================================================
    Name       Type                Description
    ========== =================== ====================================================
    tot        int                 Total RAM in **KB**
    used       int                 Total used RAM in **KB**
    free       int                 Free RAM in **KB**
    buffers    int                 Buffered RAM in **KB**
    cached     int                 Cached RAM in **KB**
    shared     int                 Shared RAM in **KB**, for NVIDIA Jetson the RAM used from GPU
    lfb        int                 Large Free Block in **4MB**
    ========== =================== ====================================================
    """
    lfb_status = ram['lfb']
    tot_ram, divider, unit_name = size_min(
        ram.get('tot', 0), start='K')
    # Make ram diagnostic status
    d_ram = DiagnosticStatus(
        name='jetson_stats {type} ram'.format(type=dgtype),
        message='{use:2.1f}{unit_ram}B/{tot:2.1f}{unit_ram}B (lfb {nblock}x4MB)'.format(
            use=ram['used'] / divider,
            unit_ram=unit_name,
            tot=tot_ram,
            nblock=lfb_status),
        hardware_id=hardware,
        values=[
            KeyValue(key="Use", value=str(ram.get('used', 0))),
            KeyValue(key="Shared", value=str(ram.get('shared', 0))),
            KeyValue(key="Total", value=str(ram.get('tot', 0))),
            KeyValue(key="Unit", value='K'),
            KeyValue(key="lfb-nblock", value=str(lfb_status)),
            KeyValue(key="lfb-size", value=str(4)),
            # TODO check if it units shoud be KB and MB, before it was just K/M so leaving as is for now
            KeyValue(key="lfb-unit", value=str('M'))])
    return d_ram


def swap_status(hardware, swap, dgtype):
    """
    Make a swap diagnostic message

    Fields:
    ========== =================== ====================================================
    Name       Type                Description
    ========== =================== ====================================================
    tot        int                 Total SWAP in **KB**
    used       int                 Total used SWAP in **KB**
    cached     int                 Cached RAM in **KB**
    table      dict                Dictionary with all swap available
    ========== =================== ====================================================
    """
    swap_cached = swap.get('cached', '0')
    tot_swap, divider, unit = size_min(
        swap.get('tot', 0), start='K')
    message = '{use}{unit_swap}B/{tot}{unit_swap}B (cached {cached}KB)'.format(
        use=swap.get('used', 0) / divider,
        tot=tot_swap,
        unit_swap=unit,
        cached=swap_cached)
    # Make swap diagnostic status
    d_swap = DiagnosticStatus(
        name='jetson_stats {type} swap'.format(type=dgtype),
        message=message,
        hardware_id=hardware,
        values=[
            KeyValue(key="Use", value=str(swap.get('used', 0))),
            KeyValue(key="Total", value=str(swap.get('tot', 0))),
            KeyValue(key="Unit", value='K'),
            KeyValue(key="Cached-Size",
                     value=str(swap_cached)),
            KeyValue(key="Cached-Unit", value='K')])
    return d_swap


def power_status(hardware, power):
    """
    Make a Power diagnostic message

    Fields:
    ============= =================== ====================================================
    Name          Type                Description
    ============= =================== ====================================================
    rail          dict                A dictionary with all thermal rails
    tot           dict                Total estimate board power
    ============= =================== ====================================================

    For each rail there are different values available

    ============= =================== ====================================================
    Name          Type                Description
    ============= =================== ====================================================
    online        bool                If sensor is online
    type          str                 Type of sensors (For NVIDIA Jetson is INA3221)
    status        str                 *(if available)* Status sensor
    volt          int                 Gets rail voltage in millivolts
    curr          int                 Gets rail current in milliamperes
    power         int                 Gets rail power in milliwatt
    avg           int                 Gets rail power average in milliwatt
    warn          int                 *(if available)* Gets rail average current limit in milliamperes
    crit          int                 *(if available)* Gets rail instantaneous current limit in milliamperes
    ============= =================== ====================================================
    """
    values = []

    # Make list for each power rail
    for rail_name in sorted(power.get('rail', {})):
        value = power['rail'][rail_name]
        watt_name = rail_name.replace("VDD_", "").replace("POM_", "").replace("_", " ")
        curr = value.get('curr', 0)
        avg = value.get('avg', 0)
        values += [
            KeyValue(key="Name", value=watt_name),
            KeyValue(key="Current Power", value=str(int(curr))),
            KeyValue(key="Average Power", value=str(int(avg)))
        ]

    # sum all rails for true total
    tot_curr = sum(r.get('curr', 0) for r in power.get('rail', {}).values())
    tot_avg  = sum(r.get('avg',  0) for r in power.get('rail', {}).values())

    d_volt = DiagnosticStatus(
        name='jetson_stats power',
        message='curr={curr}mW avg={avg}mW'.format(curr=int(tot_curr), avg=int(tot_avg)),
        hardware_id=hardware,
        values=values
    )

    return d_volt

def temp_status(hardware, temp, level_options):
    """
    Make a temperature diagnostic message

    Fields:
    ============= =================== ====================================================
    Name          Type                Description
    ============= =================== ====================================================
    online        bool                If sensor is online
    temp          int                 Gets rail voltage in Celsius. *(If offline show -256)*
    max           int                 *(if available)* Gets rail average current limit in Celsius
    crit          int                 *(if available)* Gets rail instantaneous current limit in Celsius
    ============= =================== ====================================================
    """
    values = []
    level = DiagnosticStatus.OK
    list_options = sorted(level_options.keys(), reverse=True)
    max_temp = 20
    # List all temperatures
    for key, value in temp.items():
        if not value['online']:
            pass

        values += [KeyValue(key=key, value=str(value['temp']))]
        if value['temp'] > max_temp:
            # Add last high temperature
            max_temp = value['temp']
    # Make status message
    for th in list_options:
        if max_temp >= th:
            level = level_options[th]
            break

    if level is not DiagnosticStatus.OK:
        max_temp_names = []
        # List off names
        for key, value in temp.items():
            if value['temp'] >= th:
                # Store name
                max_temp_names += [key]
        # Write a message
        message = '[' + ', '.join(max_temp_names) + \
            '] are more than {temp} C'.format(temp=th)
    else:
        message = '{n_temp} temperatures reads'.format(n_temp=len(temp))
    # Make temperature diagnostic status
    d_temp = DiagnosticStatus(
        level=level,
        name='jetson_stats temp',
        message=message,
        hardware_id=hardware,
        values=values)
    return d_temp


def emc_status(hardware, emc, dgtype):
    """
    Make a EMC diagnostic message

    Fields:
    ========== =================== ====================================================
    Name       Type                Description
    ========== =================== ====================================================
    online     bool                Status EMC
    val        int                 Percentage of bandwidth used relative to running frequency
    cur        int                 Current working frequency in **kHz**
    max        int                 Max EMC frequency usable in **kHz**
    min        int                 Min EMC frequency usable in **kHz**
    ========== =================== ====================================================
    """
    # Make EMC diagnostic status
    d_emc = DiagnosticStatus(
        name='jetson_stats {type} emc'.format(type=dgtype),
        message='{val}%'.format(val=emc['val']),
        hardware_id=hardware,
        values=[
            KeyValue(key='Val', value=str(emc['val'])),
            KeyValue(key="Freq", value=str(emc['cur'])),
            KeyValue(key="Unit", value="khz")])
    return d_emc
# EOF
