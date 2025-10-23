# idtp9418 wireless charger driver for xiaomi tablets

Tested on xiaomi-nabu on 6.16.0-sm8150, based on the downstream xiaomi driver.

Expected output:
```
idtp9418: loading out-of-tree module taints kernel.
idtp9418 3-003b: [idtp] Pen attached! Turning on reverse charging... 
idtp9418 3-003b: idtp9418_irq_work: Entered irq work
idtp9418 3-003b: Irq got: 0
idtp9418 3-003b: set reverse fod: 500
idtp9418 3-003b: tx data(0078): 0x1
idtp9418 3-003b: start reverse charging
idtp9418 3-003b: reverse charging start success
idtp9418 3-003b: [idtp] Pen detached! Turning off reverse charging...
```
# TODO:

- [x] Move from legacy gpio to gpiod for resource allocation
- [x] Set an alarm to monitor and limit charge
- [x] Register a power-supply to report state of charge to userspace and change charge limit

## Bonus stuff:
- [ ] Load a default fod program if it doesn't exist
- [ ] Get MAC address of attached pen