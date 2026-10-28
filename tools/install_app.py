"""Install a verified app into the inactive OTA slot without erasing user settings.
One serial connection covers programming and first boot, avoiding early rollback
caused by reconnecting USB while the new app is still pending verification.
"""
import argparse,binascii,hashlib,json,struct,time
from pathlib import Path
from esptool import cmds
from esptool.reset import HardReset

parser=argparse.ArgumentParser()
parser.add_argument('--port',required=True)
parser.add_argument('--expected-mac',required=True)
parser.add_argument('--firmware',type=Path,required=True)
parser.add_argument('--sha256',required=True)
parser.add_argument('--partition-table',type=Path,required=True)
parser.add_argument('--output-dir',type=Path,required=True)
parser.add_argument('--monitor-seconds',type=int,default=90)
parser.add_argument('--baud',type=int,choices=(115200,460800),default=460800)
args=parser.parse_args()
firmware=args.firmware.read_bytes()
assert hashlib.sha256(firmware).hexdigest()==args.sha256,'Firmware checksum mismatch'
assert len(firmware)<=0x300000 and firmware[0]==0xe9,'Unexpected firmware size/header'
args.output_dir.mkdir(parents=True,exist_ok=True)
esp=cmds.detect_chip(args.port)
report={'firmware_sha256':args.sha256,'app_bytes':len(firmware),'baud':args.baud,'result':'started'}
try:
    mac=':'.join(f'{b:02x}' for b in esp.read_mac())
    assert mac==args.expected_mac.lower(),'USB device identity mismatch'
    assert esp.CHIP_NAME=='ESP32-S3','Wrong chip'
    esp=esp.run_stub();esp.change_baud(args.baud)
    table=cmds.read_flash(esp,0x8000,0xc00,no_progress=True)
    assert table==args.partition_table.read_bytes(),'Device partition table differs from release'
    nvs=cmds.read_flash(esp,0x9000,0x6000,no_progress=True)
    ota=cmds.read_flash(esp,0xf000,0x2000,no_progress=True)
    (args.output_dir/'install-before-nvs.bin').write_bytes(nvs)
    (args.output_dir/'install-before-otadata.bin').write_bytes(ota)
    entries=[]
    for i in range(2):
        seq,state,crc=struct.unpack_from('<I20xII',ota,i*4096)
        valid=seq not in (0,0xffffffff) and state not in (3,4) and crc==binascii.crc32(struct.pack('<I',seq),0xffffffff)&0xffffffff
        if valid:entries.append((seq,state,i))
    assert entries,'No valid OTA selection; refusing to guess active app'
    assert len({row[0] for row in entries}) == len(entries), 'Ambiguous equal OTA sequences; inspect before installing'
    seq,state,entry=max(entries)
    assert seq < 0xfffffffe, 'OTA sequence exhausted; a separately reviewed recovery is required'
    assert state in (2,0xffffffff),'Current app is not confirmed valid; refusing upgrade'
    active=(seq-1)%2;target=1-active;offset=0x20000+target*0x300000
    next_seq=seq+1;assert (next_seq-1)%2==target
    selector=bytearray(b'\xff'*4096)
    struct.pack_into('<I',selector,0,next_seq)
    struct.pack_into('<I',selector,24,0)  # ESP_OTA_IMG_NEW, bootloader handles pending/rollback.
    struct.pack_into('<I',selector,28,binascii.crc32(struct.pack('<I',next_seq),0xffffffff)&0xffffffff)
    selector_offset=0xf000+(1-entry)*4096
    (args.output_dir/'install-selector-sector.bin').write_bytes(selector)
    report.update({'old_slot':active,'new_slot':target,'app_offset':hex(offset),'selector_offset':hex(selector_offset),'nvs_before_sha256':hashlib.sha256(nvs).hexdigest()})
    # First write and verify app; only then change the boot selector.
    cmds.write_flash(esp,[(offset,firmware)],compress=True)
    cmds.verify_flash(esp,[(offset,firmware)])
    assert cmds.read_flash(esp,0x9000,0x6000,no_progress=True)==nvs,'NVS changed during app flash'
    cmds.write_flash(esp,[(selector_offset,bytes(selector))],compress=True)
    cmds.verify_flash(esp,[(selector_offset,bytes(selector))])
    report.update({'flash_verified':True,'nvs_unchanged_by_installer':True,'result':'programmed_waiting_boot'})
    (args.output_dir/'usb-install.json').write_text(json.dumps(report,indent=2)+'\n')
    # Native USB JTAG/serial needs the USB reset release delay on this Mac.
    esp.write_reg(esp.RTC_CNTL_OPTION1_REG, 0, esp.RTC_CNTL_FORCE_DOWNLOAD_BOOT_MASK)
    HardReset(esp._port, uses_usb=esp.uses_usb_jtag_serial() or esp.uses_usb_otg())()
    esp._port.baudrate=115200;esp._port.timeout=.3
    end=time.monotonic()+args.monitor_seconds
    with (args.output_dir/'first-boot.log').open('wb') as out:
        while time.monotonic()<end:
            data=esp._port.read(max(1,esp._port.in_waiting))
            if data:out.write(data);out.flush()
    report['result']='programmed_and_boot_log_captured'
    (args.output_dir/'usb-install.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
finally:
    esp._port.close()
