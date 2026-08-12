# Thor H2 network provisioning

These files reproduce Thor's persistent `.125` workstation link, routed `.123`
H2 path, and H2 -> Thor -> workstation clock chain. Run the commands in this
document **on Thor**, from an `xr_teleoperate` checkout of the same deployed
branch/commit as the workstation.

The installer intentionally uses an interface name and the profile currently
active on that interface. It does not copy a NetworkManager UUID or MAC address
from the original Thor. It saves the profile without reactivating it by default,
so provisioning over SSH does not unexpectedly tear down that SSH connection.

## Repository files and installed destinations

| Repository file | Thor destination | Purpose |
|---|---|---|
| `install_thor_usb_ethernet.sh` | run from the checkout; not installed | Create/update the persistent NetworkManager profile |
| `h2-dds-routing` | `/usr/local/sbin/h2-dds-routing` | Restore the `.123.163` alias, route, forwarding, proxy ARP, and scoped firewall rules |
| `h2-dds-routing.service` | `/etc/systemd/system/h2-dds-routing.service` | Run routing setup after networking at every boot |
| `50-h2-source.conf` | `/etc/chrony/conf.d/50-h2-source.conf` | Make H2 `192.168.123.161` Thor's NTP source |
| `60-h2-workstation.conf` | `/etc/chrony/conf.d/60-h2-workstation.conf` | Allow only workstation `192.168.125.222` to use Thor as NTP server |

## Install

The verified USB Ethernet interface is `enx80691a14d263`. Preview the intended
NetworkManager settings, then save them:

```bash
cd ~/xr_teleoperate
teleop/deploy/thor/install_thor_usb_ethernet.sh --dry-run
sudo teleop/deploy/thor/install_thor_usb_ethernet.sh
```

The resulting profile owns `192.168.125.163/24`, autoconnects, supplies no
default gateway or automatic DNS, and has `ipv4.never-default=yes`. If Thor
enumerates the adapter under another name, pass it explicitly:

```bash
sudo teleop/deploy/thor/install_thor_usb_ethernet.sh \
  --interface THOR_USB_INTERFACE
```

Install the routing and Chrony files:

```bash
sudo install -D -m 0755 \
  teleop/deploy/thor/h2-dds-routing \
  /usr/local/sbin/h2-dds-routing
sudo install -D -m 0644 \
  teleop/deploy/thor/h2-dds-routing.service \
  /etc/systemd/system/h2-dds-routing.service
sudo install -D -m 0644 \
  teleop/deploy/thor/50-h2-source.conf \
  /etc/chrony/conf.d/50-h2-source.conf
sudo install -D -m 0644 \
  teleop/deploy/thor/60-h2-workstation.conf \
  /etc/chrony/conf.d/60-h2-workstation.conf
```

If an earlier setup added `server 192.168.123.161 iburst` directly to
`/etc/chrony/chrony.conf`, remove that one hand-edited copy after installing the
drop-in. Keep exactly one H2 source declaration, in
`/etc/chrony/conf.d/50-h2-source.conf`:

```bash
grep -RFn 'server 192.168.123.161 iburst' \
  /etc/chrony/chrony.conf /etc/chrony/conf.d
```

Thor's `/etc/chrony/chrony.conf` must contain a `confdir
/etc/chrony/conf.d` directive. The stock Thor Chrony configuration already has
it; verify it rather than copying a whole machine-specific `chrony.conf` into
Git:

```bash
grep -F 'confdir /etc/chrony/conf.d' /etc/chrony/chrony.conf
```

When a non-default USB interface was passed to the installer, the routing
script needs the same name. After installing the service file, run:

```bash
sudo systemctl edit h2-dds-routing.service
```

Enter the following, replacing `THOR_USB_INTERFACE`:

```ini
[Service]
Environment=WORKSTATION_INTERFACE=THOR_USB_INTERFACE
```

Enable the routing service, then reboot. A reboot is the safest way to prove
that NetworkManager, Chrony, and routing recover in their real startup order:

```bash
sudo systemctl daemon-reload
sudo systemctl enable h2-dds-routing.service
sudo reboot
```

For local-console provisioning only, `--activate` may be passed to the network
installer instead of rebooting. The installer refuses to reactivate the link
when the current SSH session terminates at `192.168.125.163`.

## Post-reboot verification

Run these checks on Thor after H2 and the workstation link are connected:

```bash
ip -4 address show dev enx80691a14d263
ip -4 address show dev eth10
ip route show 192.168.123.222/32
sysctl net.ipv4.ip_forward net.ipv4.conf.eth10.proxy_arp
systemctl is-enabled h2-dds-routing.service
systemctl is-active h2-dds-routing.service chrony.service
chronyc sources -v
chronyc tracking
ping -c 2 192.168.123.161
ping -c 2 192.168.125.222
```

Expected results:

- USB Ethernet has `192.168.125.163/24` and `eth10` has
  `192.168.123.163/24`;
- `192.168.123.222/32` is link-scoped through USB Ethernet;
- forwarding and `eth10` proxy ARP both report `1`;
- both services are active and the routing service is enabled;
- Chrony lists `192.168.123.161` and selects it after its normal settling time;
- both H2 and workstation pings succeed.

When a non-default USB interface was selected, substitute it in the commands.
From the workstation, finish the end-to-end check with:

```bash
ip route get 192.168.123.161 from 192.168.123.222
ssh -o BatchMode=yes unitree@192.168.125.163 true
timedatectl timesync-status
```

The workstation route must use `192.168.125.163` as gateway and
`192.168.123.222` as source. Do not start robot control until these checks pass.
