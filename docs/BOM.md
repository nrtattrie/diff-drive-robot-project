# Bill of Materials

Last Updated: 2026-01-18

## Summary

| Category | Estimated Cost |
|----------|----------------|
| Already Owned | $180 |
| Phase 1 (Basic Robot) | $100 |
| Phase 2 (SLAM) | $100 |
| Phase 3 (Advanced) | $50 |
| **Grand Total** | **~$430** |

---

## Electronics - Owned

| Component | Qty | Status | Cost | Location | Specs | Notes |
|-----------|-----|--------|------|----------|-------|-------|
| Raspberry Pi 5 (8GB) | 1 | ✅ Have | $80 | Desk | 2.4GHz quad-core Cortex-A76 | Main computer |
| M.2 HAT | 1 | ✅ Have | $24 | Desk | PCIe Gen 2 ×1 | For NVMe SSD |
| NVMe SSD (256GB) | 1 | 🚚 Ordered | $89 | In transit | Raspberry Pi SSD | Ubuntu boot drive |
| Active Cooler | 1 | ✅ Have | $12 | Desk | PWM fan + heatsink | Thermal management |
| Arduino Mega 2560 | 1 | ✅ Have | $0 | Desk | ATmega2560 | Motor controller |
| ELEGOO Sensor Kit | 1 | ✅ Have | $66 | Desk | Multiple sensors | Ultrasonic, buttons, LEDs |

**Subtotal (Owned):** $220

---

## Electronics - To Order (Phase 1)

| Item | Qty | Priority | Est. Cost | Vendor | Part Number | Purpose |
|------|-----|----------|-----------|--------|-------------|---------|
| TBD DC Motors w/ Encoders | 2 | HIGH | $30 | Amazon | DG01D-E | Drive wheels |
| TBD Robot Wheels (65mm) | 2 | HIGH | $10 | Amazon | Match motor shaft | Traction |
| TBD Ball Caster Wheel | 1 | HIGH | $5 | Amazon | 15mm ball | Rear support |
| TBD Motor Driver | 1 | HIGH | $8* | Amazon | L298N module | H-bridge (check ELEGOO kit first) |
| TBD IMU | 1 | HIGH | $25 | Adafruit | Product ID: 2472 | 9-DOF orientation |
| TBD M3 Standoff Kit | 1 | MEDIUM | $12 | Amazon | Assorted 40-80mm | Deck mounting |
| TBD Buck Converter (5V) | 1 | MEDIUM | $8 | Amazon | LM2596 module | RPi power from battery |
| TBD USB-C Cable (short) | 1 | MEDIUM | $5 | Amazon | 15cm, right-angle | RPi power |
| TBD Dupont Wire Kit | 1 | MEDIUM | $10 | Amazon | Male-Male, Female-Female | Connections |

**Subtotal (Phase 1):** ~$100

*Check ELEGOO kit for L298N before ordering*

---

## Electronics - Phase 2 (SLAM)

| Item | Qty | Est. Cost | When | Purpose |
|------|-----|-----------|------|---------|
| RPLidar A1M8 | 1 | $99 | Month 3 | 360° laser scanner for SLAM |
| RPi Camera Module 3 | 1 | $25 | Month 4 | Computer vision |

**Subtotal (Phase 2):** ~$124

---

## Electronics - Phase 3 (Advanced/Optional)

| Item | Qty | Est. Cost | When | Purpose |
|------|-----|-----------|------|---------|
| Pololu 37D Motors | 2 | $50 | Month 3 | Upgrade to better motors |
| 18650 Battery Pack (3S) | 1 | $35 | Month 2 | Power system |
| Battery Monitor | 1 | $8 | Month 2 | Voltage/current monitoring |
| Emergency Stop Button | 1 | $6 | Month 2 | Safety |

**Subtotal (Phase 3):** ~$99

---

## Mechanical Parts

| Item | Qty | Est. Cost | Source | Notes |
|------|-----|-----------|--------|-------|
| PLA Filament (1kg) | 1 | $20 | On hand | Already have black PLA |
| M3 Heat-Set Inserts | 20 | $8 | Amazon | Brass threaded inserts |
| M3 × 8mm Screws | 30 | $5 | Amazon | Fasteners |
| M2.5 × 8mm Screws | 10 | $3 | Amazon | RPi mounting |

**Subtotal (Mechanical):** ~$36 (mostly have)

---

## Tools/Consumables

| Item | Status | Notes |
|------|--------|-------|
| Bambu X1-C 3D Printer | ✅ Have | For all structural parts |
| Soldering Iron | ✅ Have | For heat-set inserts |
| Multimeter | ✅ Have | Debugging |
| Wire Stripper/Crimper | Need | ~$15 |

---

## Comparison to Commercial Options

| Option | Cost | Capabilities |
|--------|------|--------------|
| **This Build** | ~$430 | Full autonomous navigation, customizable |
| TurtleBot3 Burger | $1,295 | Similar capabilities, less customizable |
| TurtleBot4 | $1,795 | Better sensors, commercial support |
| DIY Kit (no RPi/sensors) | $200-400 | Basic platform only |

**Value Proposition:** Building custom saves $800+ and provides better learning experience.

---

## Purchase Timeline

### Week 1 (Jan 13-19)
- [x] NVMe SSD - Ordered Jan 17

### Week 2 (Jan 20-26)
- [ ] Phase 1 electronics (motors, wheels, IMU, etc.)
- [ ] Mechanical fasteners

### Week 6-8 (Feb 24 - Mar 9)
- [ ] RPLidar A1M8 (after basic robot working)

### Month 4+ (May+)
- [ ] Camera module
- [ ] Optional upgrades as needed

---

## Datasheets & References

All datasheets stored in: `Google Drive > Robotics Project > Datasheets/`

- [Raspberry Pi 5 Datasheet](https://datasheets.raspberrypi.com/rpi5/raspberry-pi-5-product-brief.pdf)
- [Arduino Mega Pinout](https://docs.arduino.cc/hardware/mega-2560/)
- [BNO055 Datasheet](https://www.adafruit.com/product/2472)
- [RPLidar A1 Manual](https://www.slamtec.com/en/Lidar/A1)

---

## Notes

- All prices in USD
- Prices are estimates, may vary
- Check ELEGOO kit inventory before ordering duplicates
- Consider buying from single vendor (Amazon) to save shipping
- Keep receipts in `Google Drive > Robotics Project > Receipts/`
