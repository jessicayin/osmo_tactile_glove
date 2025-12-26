# Bill of Materials

This is for making 1 x OSMO tactile glove. 

## Sensor Assembly

### Electronics
| Item | Quantity | Purpose | Link to Purchase | Price |
|------|----------|---------|------------------|-------|
| MCU board | 1 | Glove microcontroller | PCBWay.com | config dependent|
| Sensor boards | 12 | Tactile + IMU sensors | PCBWay.com | config dependent |
| Enameled 28 AWG magnet wire | 1 | Connect sensors to MCU | [Amazon](https://a.co/d/eH0bxc9) | $8.79 |
| MuMetal sheet, 0.006" x 8" x 12" | 1 | Crosstalk reduction | [Amazon](https://a.co/d/j9fOF7b) | $79.99 |
| Silpoxy adhesive, 3oz. | 1 | Sensor assembly | [Amazon](https://a.co/d/j6YFYTj) | $32.88 |

In the paper, we use braided 4 strand tinsel wire from [Maeden Innovation Co.](https://www.maeden.com/en/flexiblewire/tinselwire) but we suggest the enameled magnet wire as a more affordable and accessible substitute. If interested in using the Maeden tinsel wire, here is the info: Part No. CAG25D53YA, Lot No. 21081105, N.W. > 79, 5 conductor.

### Magnetic Elastomers
| Item | Quantity | Purpose | Link to Purchase | Price |
|------|----------|---------|------------------|-------|
| Magnequench MQFP-15-7 magnetic particles | 20g | Embedded in magnetic elastomer | [Magnequench](https://mqitechnology.com/) | - |
| Smooth-On 00-30 Platinum Elastomer, 2.0lb Trial Kit | 1 | Elastomer base | [Amazon](https://a.co/d/4OK6vdy) | $36.02 |
| 3D Printed Mold - Fingertips | 1 | Mold fingertip magnetic skins| - | - |
| 3D Printed Mold - Palm | 1 | Mold palm magnetic skins | - | - |

Please refer to the [AnySkin](https://any-skin.github.io/) work for more details on fabrication.

For the 3D printed molds, we recommend a high-resolution resin print for minimal texture on the magnetic patches. We used the Stratasys Objet with VeroWhite in this work. TODO: upload 3d print mold files

#### Magnetic Particles FAQ
Update 12/21/25

You can place an order by contacting Magnequench here: https://mqitechnology.com/support-contact/contact-us/ and work with a sales rep, they usually respond quickly (within 1-2 days in my experience) and are helpful with technical advice. We use MQFP-15-7 (25µM) - make sure NOT to order MQP (missing "F" in the product name). The difference is the particle size, and MQFP particles are about 100x smaller than MQP particles, which makes a huge difference in uniform distribution while curing the magnetic skins (source: [AnySkin](https://any-skin.github.io/)). It has been some time since we've ordered but the lead time was ~2-3 weeks from Thailand. 

### Equipment
| Item | Quantity | Purpose | Link to Purchase | Price |
|------|----------|---------|------------------|-------|
| Soldering iron | 1 | Solder components | [Amazon](https://a.co/d/9K3vRqF) | $121.47 |
| Solder (contains lead) | 1 | Join connections | [Amazon](https://a.co/d/3eHiCQU) | $22.90 |
| Plastic cups | 1 | Mix elastomers | [Amazon](https://a.co/d/htiVjTC) | $12.99 |
| Cotton tipped applicators | 5 | Apply adhesive / stir stick | [Amazon](https://a.co/d/b7sryC6) | $7.59 |
| Mold release spray | 1 | Release magnetic elastomers from molds | [Amazon](https://a.co/d/cohaXUb) | $19.80 |
| Multimeter | 1 | Verify wiring and troubleshooting | [Amazon](https://a.co/d/f0VvZzt) | $13.95 |
| Sandpaper, coarse grits | 1 | Surface prep | [Amazon](https://a.co/d/g1jqryB) | $5.14 |
| Magnet-Physik pulse magnetizer | 1 | Magnetize elastomers | - | - |
| Scale, grams | 1 | Measure elastomer and magnet particle ratios | [Amazon](https://a.co/d/6x2ZWb1) | $9.99 |
| Chipquik SMD4300TF30 flux (Optional) | 1 | Significantly improve soldering quality | [Digikey](https://www.digikey.com/en/products/detail/chip-quik-inc/SMD4300TF30/7035053) | $39.99 |
| UV glue + UV flashlight (Optional) | 1 | Strengthen solder joints | [Amazon](https://a.co/d/7nifiRX) | $12.74 |
| Xacto knife (Optional) | 1 | Trim magnetic patches | [Amazon](https://a.co/d/iCEcXmX) | $8.89 |


#### Pulse Magnetizer FAQ
Updated 12/26/25

This is the specific pulse magnetizer: https://www.magnet-physik.de/wp-content/uploads/2023/02/38565336-IM-K-e-3171.pdf from Magnet-Physik. Magnet-Physik also offers a magnetizing service for batches, and you can reach out to them via their website. If you have access to university resoures, consider reaching out to academic labs studying materials science / phyiscs / related sciences and asking if they have this machine.

When magnetizing the magnetic skins, they should be stacked vertically (i.e., one skin directly on top of another skin) and located in the center of the chamber, which will impart the greatest possible magnetization. I approximated this by eye and stacked them neatly by hand so unfortunately I don't have any specific fixtures to share.

Magnetization settings:
- Output 1
- Max Energy: 10000 Ws
- Max Voltage: 2000 V (max value)
- Continuous Output: 100 W
- Max Impulse Current: 5.0 kA
- IP Code: IP 20
- Cycle Time: 12.0 s
- Max Temperature: 50 C
- Function/Waveform A

## Glove Assembly
| Item | Quantity | Purpose | Link to Purchase | Price |
|------|----------|---------|------------------|-------|
| Beige gloves | 2 | Base + outer glove | [Amazon](https://a.co/d/8hkbJJY) | $7.99 |
| Silpoxy silicone adhesive | 1 | Attach sensors | [Amazon](https://a.co/d/j6YFYTj) | $32.88 |
| Kapton tape | 1 | Electrical insulation, assembly aid | [Amazon](https://a.co/d/6VlnjA4) | $9.99 |
| STLinkv3 STM32 Programmer | 1 | Program MCU | [Amazon](https://a.co/d/0UaakWi) | $72.50 |
| TagConnect TC2030-IDC-NL | 1 | Program MCU | [Tag-Connect](https://www.tag-connect.com/product/tc2030-idc-nl) | $33.95 |
| USB C to USB A cable, 10ft | 1 | Glove power and data transfer | [Amazon](https://a.co/d/fmYgTqv) | $9.99 |
| Laser cut acrylic stencil of hand (Optional) | 1 | Assembly aid | - | - |



