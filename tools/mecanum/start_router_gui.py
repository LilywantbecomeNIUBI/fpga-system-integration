"""Offline LAN launcher: select the current FPGA route and configure PC replies."""
from pathlib import Path
import argparse,ipaddress,json,socket,subprocess,sys,time
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from tools.task4.telemetry_endpoint import transact

def lan_address(board):
    board=str(ipaddress.IPv4Address(board))
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as route:
        route.connect((board,6206))
        local=route.getsockname()[0]
    network=ipaddress.IPv4Network(board+'/24',strict=False)
    if ipaddress.IPv4Address(local) not in network or local==board:
        raise RuntimeError('请先连接车载路由器的 Wi-Fi 或 LAN 网线；电脑需要取得 '+str(network)+' 内的独立地址。当前到板端的出口为 '+local)
    return local

def wifi_route(profile,board,seconds=30):
    joined=subprocess.run(['netsh','wlan','connect','name='+profile],capture_output=True,text=True,errors='replace')
    if joined.returncode:raise RuntimeError('无法连接已保存的 Wi-Fi 配置 '+profile+'：'+joined.stdout+joined.stderr)
    print('正在连接 '+profile+'；无 Internet 不影响本地控车。',flush=True)
    last_error='等待 WLAN 地址'
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        query=subprocess.run(['powershell','-NoProfile','-Command',
            "@(Get-NetIPAddress -InterfaceAlias 'WLAN' -AddressFamily IPv4 -ErrorAction SilentlyContinue | Select-Object -ExpandProperty IPAddress) | ConvertTo-Json -Compress"],
            capture_output=True,text=True,errors='replace')
        addresses=json.loads(query.stdout.strip() or '[]')
        if isinstance(addresses,str):addresses=[addresses]
        try:
            local=lan_address(board)
            if local in addresses:return local
            last_error='当前出口 '+local+' 不是 WLAN；请先拔掉电脑的有线网线，再运行无线入口'
        except (OSError,ValueError,RuntimeError) as exc:last_error=str(exc)+'；WLAN 地址：'+str(addresses)
        time.sleep(1)
    raise RuntimeError(last_error)

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--fpga-ip',default='192.168.50.30')
    ap.add_argument('--wifi-profile',help='Explicitly connect this saved Windows Wi-Fi profile before starting')
    ap.add_argument('--detect-only',action='store_true',help='Read local route only; no packets, endpoint changes or GUI')
    args=ap.parse_args()
    try:
        if args.detect_only and args.wifi_profile:raise ValueError('--detect-only 不会切换 Wi-Fi，请去掉 --wifi-profile')
        local=wifi_route(args.wifi_profile,args.fpga_ip) if args.wifi_profile else lan_address(args.fpga_ip)
        print('当前控车接口：'+local+' → FPGA '+args.fpga_ip,flush=True)
        if args.detect_only:return
        print('配置 PC 状态回传；Nano 端点保持原值。无需互联网。',flush=True)
        state=transact(local,board=args.fpga_ip,target=local,slot='pc')
        if state['target']!=local or not state['enabled']:raise RuntimeError('板端未确认 PC 状态目标')
        print('配置成功。打开后点击连接设备；不会自动使能或开始运动。',flush=True)
        completed=subprocess.run([sys.executable,str(ROOT/'host/mecanum/gui.py'),'--transport','router',
                                  '--telemetry-bind',local,'--fpga-ip',args.fpga_ip],cwd=ROOT)
        if completed.returncode:raise RuntimeError('上位机退出，错误码 '+str(completed.returncode))
    except (OSError,ValueError,RuntimeError,TimeoutError) as exc:
        print('启动失败：'+str(exc),file=sys.stderr)
        print('切换有线/Wi-Fi后，请关闭旧上位机，再重新运行本启动器。',file=sys.stderr)
        raise SystemExit(1)

if __name__=='__main__':main()
