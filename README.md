# analog-toolkit
A small and simple STM32G431 based dev board, doubling as a simple analog-to-USB acquisition tool.

Next to 6 single ended or 2 differential ADC inputs, there are 3 PGA connected ADC channels, 3 general purpose IO, 2 DAC outputs, 2 Timer outputs, and several protocols such as UART, I2C, USB FS and USB-PD. All ADCs have a standard input resolution of 12 bits, with hardware oversampling up to 16 bits, the DAC has 12 bits resolution. 
Equiped with a DFU bootloader, this board can simply be flashed from USB. All Ports are available through standard 2.54mm pinheader, a scope aligator clip can be attached to the grounding tab. A RGB led allows for direct status indication.

<table>
  <tbody>
    <tr>
      <td>
        <img src="/front.jpg" title="A white pcb reading “analog toolkit”, where the a is an anarchy a. There is a MCU, USB, pin headers and a led"/>
      </td>
      <td>
        <img src="/back.jpg" title="Back side of the pcb with pin annotations, and some more informations on the hw, as well as silkscreen art"/>
      </td>
    </tr>
    <tr>
      <td colspan="2">
        <img src="render.png" title="render of front side of pcb"/>
      </td>
    </tr>
  </tbody>
</table>

## folder structure
```
 /hardware          - PCB files
 /ADC               - ADC sampling firmware, with selectable samplerate and API
 /ADC/software      - python receiving sketch for ADC firmware
 /DAC/              - DAC firmware bit, this one doesn't do much yet
 /pd-dev/           - USB-PD Sink firmware
```

## things I want to do and explore

 - HW
 - [x] replace RGB led with two discrete LEDs
 - [ ] cheaper JLC version, keep functionality
   - [ ] button -> C720477
   - [ ] LDO -> C14289
   - [ ] USBLC -> C32677
   - [ ] RGB LED -> two discrete, C34499, C2296
 
 - SW/FW
 - [ ] build CLI
   - [ ] build basic CLI firmware
 - [ ] improve ADC fw
   - [ ] add interactive python console (60% done)
   - [ ] PGA current and temperature sensing
   - [ ] Implement byte stuffing
 - [ ] improve DAC fw
   - [ ] make sample rate settable
   - [ ] make sample settable
 - [ ] Implement Sigrok protocol
 - [ ] Implement SCPI
 - [x] USB-PD firmware
 
## license
 
Copyright Jana Marie Hemsing 2026.

This source describes Open Hardware and is licensed under the CERN-OHL-S v2.

You may redistribute and modify this source and make products using it under the terms of the CERN-OHL-S v2 (https://ohwr.org/cern_ohl_s_v2.txt).

This source is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY, INCLUDING OF MERCHANTABILITY, SATISFACTORY QUALITY AND FITNESS FOR A PARTICULAR PURPOSE. Please see the CERN-OHL-S v2 for applicable conditions.

Source location: https://github.com/Jana-Marie/analog-toolkit

As per CERN-OHL-S v2 section 4, should You produce hardware based on this source, You must where practicable maintain the Source Location visible on the external case of the Gizmo or other products you make using this source.
