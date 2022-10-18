$fn = 50;
sizePlate=175;
xPart1=(sizePlate-76)/2+0.5;

translate([0,40,0]){
difference(){
    union(){
        translate([0,-40,0])cube([sizePlate,sizePlate,4]);//Grundplatte
        translate([xPart1,0,4])cube([5,55,45]);
        translate([xPart1-1+71,0,4])cube([5,55,45]);
        translate([xPart1+5,0,35])cube([5,55,14]);
        translate([xPart1-1+61+5,0,35])cube([5,55,14]);
        }

    //Löcher Seite 1
    union(){
            translate([xPart1,5.5,49-2.5-1])rotate([0,90,0])cylinder(10,2.5,2.5);//
            
            translate([xPart1,5.5,49-2.5-1-7])rotate([0,90,0])cylinder(10,2.5,2.5);
            
            translate([xPart1,5.5+22,49-2.5-1])rotate([0,90,0])cylinder(10,2.5,2.5);
            
            translate([xPart1,5.5+22,49-2.5-1-7])rotate([0,90,0])cylinder(10,2.5,2.5);
            
            translate([xPart1,5.5+44,49-2.5-1])rotate([0,90,0])cylinder(10,2.5,2.5);
            
            translate([xPart1,5.5+44,49-2.5-1-7])rotate([0,90,0])cylinder(10,2.5,2.5);
        }
        
        //Löcher Seite 2
        xholes = xPart1+66-1;
    union(){
            translate([xholes,5.5,49-2.5-1])rotate([0,90,0])cylinder(10,2.5,2.5);
            
            translate([xholes,5.5,49-2.5-1-7])rotate([0,90,0])cylinder(10,2.5,2.5);
            
            translate([xholes,5.5+22,49-2.5-1])rotate([0,90,0])cylinder(10,2.5,2.5);
            
            translate([xholes,5.5+22,49-2.5-1-7])rotate([0,90,0])cylinder(10,2.5,2.5);
            
            translate([xholes,5.5+44,49-2.5-1])rotate([0,90,0])cylinder(10,2.5,2.5);
            
            translate([xholes,5.5+44,49-2.5-1-7])rotate([0,90,0])cylinder(10,2.5,2.5);
        }
    }}