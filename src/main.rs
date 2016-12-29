#![allow(dead_code)]
#[macro_use]
extern crate scad_generator;
extern crate scad_util;
extern crate nalgebra as na;

use scad_generator::*;

use std::string::String;

use scad_util::add_named_color;

const SCREW_DIAMETER: f32 = 3.5;

fn get_m3_screw(length: f32) -> ScadObject
{
    let screw_padding = 0.5;
    let head_padding = 1.;
    let diameter = 3. + screw_padding;
    let head_diameter = 6. + head_padding;
    let head_height = 3.;

    scad!(Union;
    {
        scad!(Cylinder(head_height, Diameter(head_diameter))),
        scad!(Cylinder(length + head_height, Diameter(diameter))),
    })
}

qstruct!(NazeBoard()
{
    hole_diameter: f32 = 3.,
    width: f32 = 36.,
    hole_distance: f32 = 30.,
    hole_padding_radius: f32 = hole_diameter / 2. + 1.5
});

impl NazeBoard
{
    fn get_board(&self) -> ScadObject
    {
        let height = 3.5;
        let main = scad!(Hull;
        {
            self.place_object_at_holes(
                scad!(Cylinder(height, Radius(self.hole_padding_radius)))
            )
        });

        let holes = self.get_holes(height);

        scad!(Difference;{
            main,
            holes
        })
    }

    fn get_holes(&self, height: f32) -> ScadObject
    {
        self.place_object_at_holes(
                scad!(Cylinder(height, Diameter(self.hole_diameter)))
            )
    }

    fn place_object_at_holes(&self, object: ScadObject) -> ScadObject
    {
        let mut result = scad!(Union);
        for x in vec!(-self.hole_distance / 2., self.hole_distance / 2.)
        {
            for y in vec!(-self.hole_distance / 2., self.hole_distance / 2.)
            {
                let translated = scad!(Translate(vec3(x, y, 0.));{
                    object.clone()
                });

                result.add_child(translated);
            }
        }

        result
    }
}

qstruct!(BoardCamera()
{
    width: f32 = 32.,
    thickness: f32 = 4.,
    lens_diameter: f32 = 17.,
    lens_length: f32 = 24.,
});

impl BoardCamera
{
    pub fn get_model(&self) -> ScadObject
    {
        let pcb = centered_cube(
                vec3(self.width, self.width, self.thickness),
                (true, true, false)
            );

        let lens = scad!(Cylinder(
                self.thickness + self.lens_length,
                Diameter(self.lens_diameter)
            ));

        scad!(Union;
        {
            pcb,
            lens
        })
    }

    pub fn get_lens_hole(&self) -> ScadObject
    {
        let snowproofing_padding_radius = 3.;

        let total_radius = snowproofing_padding_radius + self.lens_diameter / 2.;

        scad!(Cylinder(self.lens_length, Radius(total_radius)))
    }
}


qstruct!(Esc()
{
    width: f32 = 20.,
    length: f32 = 25.,
    thickness: f32 = 4.
});

impl Esc
{
    pub fn get_pcb(&self, center: (bool, bool, bool)) -> ScadObject
    {
        let (center_x, center_y, center_z) = center;

        centered_cube(
            vec3(self.width, self.length, self.thickness),
            (center_x, center_y, center_z)
        )
    }
}



qstruct!(EscStack()
{
    layer_thickness: f32 = 3.,
    esc: Esc = Esc::new(),
    screw_padding: f32 = 2.
});

impl EscStack
{
    pub fn place_object_at_holes(&self, object: ScadObject) -> ScadObject
    {
        let mut result = scad!(Union);

        let x_position = self.esc.width / 2. 
                            + self.screw_padding / 2.
                            + SCREW_DIAMETER / 2.;
        let y_position = self.esc.length / 4.;

        let points = vec!(
                vec3(x_position, y_position, 0.),
                vec3(-x_position, y_position, 0.),
                vec3(x_position, -y_position, 0.),
                vec3(-x_position, -y_position, 0.),
            );

        for point in points
        {
            result.add_child(scad!(Translate(point);{
                object.clone()
            }));
        }

        result
    }

    pub fn get_mid_section(&self) -> ScadObject
    {
        let chamfer_radius = SCREW_DIAMETER / 2. + self.screw_padding;
        let main = scad!(Hull;{
            self.place_object_at_holes(
                    scad!(Cylinder(self.layer_thickness, Radius(chamfer_radius)))
                )
        });

        scad!(Difference;
        {
            main,
            self.place_object_at_holes(
                    scad!(Cylinder(self.layer_thickness, Diameter(SCREW_DIAMETER)))
                )
        })
    }
}


fn get_body_section(inner_width: f32, outer_width: f32, length: f32) -> ScadObject
{
    let points = vec!(
        na::Vector2::new(0., -inner_width / 2.),
        na::Vector2::new(0., inner_width / 2.),
        na::Vector2::new(length, outer_width / 2.),
        na::Vector2::new(length, -outer_width / 2.),
    );
    
    scad!(Polygon(PolygonParameters::new(points)))
}

qstruct!(TricopterBody()
{
    radius: f32 = 80.,
    height: f32 = 5.,
    outer_width: f32 = 23.,
    back_outer_width: f32 = 33.,
    inner_width: f32 = 50.,

    arm_width: f32 = 10.,

    front_block_x: f32 = 30.,

    front_section_width: f32 = inner_width,
    front_section_length: f32 = 60.,
    front_section_corner_radius: f32 = 2.,

    canopy_thickness: f32 = 3.,
    edge_thickness: f32 = canopy_thickness / 2.,
    edge_padding: f32 = 0.25,
    edge_height: f32 = 3.,

    camera_box_length: f32 = 20.,
    camera_box_edge_width: f32 = 1.,
    camera_box_edge_padding: f32 = 0.5,
    
    motor_wire_hole_radius: f32 = 4.,

    camera_offset_from_top: f32 = 10.,
});


impl TricopterBody
{
    /**
      Main function for getting the bottom section of the body
    */
    pub fn get_body_bottom(&self) -> ScadObject
    {
        //Parameters for extruding things to the height of the bottom plate
        let low_extrude_params = LinExtrudeParams
        {
            height:self.height,
            ..Default::default()
        };
        //Parameters for extruding things to the height of the plate +
        //the height of the blocky parts
        let high_extrude_params = LinExtrudeParams
        {
            height:self.get_bottom_total_height(),
            ..Default::default()
        };

        //Parameters for extruding things to the height of the camera box edge.
        let edge_extrude_params = LinExtrudeParams
        {
            height: self.get_bottom_total_height() + self.height,
            .. Default::default()
        };

        //The body without things cut out
        let body = scad!(Union;{
            scad!(LinearExtrude(low_extrude_params);
            {
                self.get_body_shape(),
                self.get_front_section()
            }),
            scad!(LinearExtrude(high_extrude_params);
            {
                self.get_back_mount_block(),
                self.get_camera_box_outline(),
            }),
            scad!(LinearExtrude(edge_extrude_params.clone());
            {
                self.get_camera_box_bottom_edge_outline()
            })
        });


        //Cutout for the camera box
        let camera_box_cutout = {
            let cutter = scad!(LinearExtrude(edge_extrude_params);
            {
                self.get_camera_box_bottom_cutout_outline()
            });

            scad!(Translate(vec3(0., 0., self.height)); cutter)
        };

        let camera_lens_hole = {
            let z_offset = self.camera_offset_from_top 
                           + self.get_bottom_total_height();

            self.get_camera_lens_hole(z_offset)
        };

        //Cutting out things like holes
        scad!(Difference;
        {
            body
            , self.get_front_arm_screw_holes()
            , self.get_back_screwholes()
            , camera_box_cutout
            , camera_lens_hole
        })
    }

    /**
      Main function for the top section of the body
    */
    pub fn get_body_top(&self) -> ScadObject
    {
        let esc_stack_offset = 40.;

        let linear_extrude = LinExtrudeParams{
            center:false,
            height:self.height,
            ..Default::default()
        };

        let body = scad!(Union;
        {
            scad!(LinearExtrude(linear_extrude.clone());
            {
                self.get_body_shape(),
                self.get_front_section(),
            }),

            scad!(Translate(vec3(0., 0., self.height));{
                self.get_top_plate_canopy_edges()
            }),
        });
        
        let camera_box = scad!(LinearExtrude(linear_extrude.clone());
            {
                self.get_camera_box_top_cutout()
            });


        let esc_stack_holes = {
            let holes = EscStack::new()
                .place_object_at_holes(get_m3_screw(self.height));

            let rotated = scad!(Rotate(90., vec3(0., 0., 1.)); holes);

            scad!(Translate(vec3(esc_stack_offset, 0., 0.)); rotated)

        };

        let screwholes = scad!(Union;
        {
            NazeBoard::new().place_object_at_holes(get_m3_screw(self.height)),
            esc_stack_holes
        });

        let with_holes = scad!(Difference;
        {
            body,
            self.get_front_arm_screw_holes(),
            self.get_back_screwholes(),
            self.get_top_plate_motor_wire_hole(),
            self.get_battery_wire_hole(),
            self.get_camera_lens_hole(self.camera_offset_from_top),
            screwholes,
            camera_box,
        });

        scad!(Intersection;{
            with_holes,
            self.get_side_bounds()
        })
    }

    /**
      Function for getting the 2d outline of the body. Does not include
      the cutoff to 14cm on the sides that is done by self.get_side_bounds()
    */
    fn get_body_shape(&self) -> ScadObject
    {
        let back_segment = get_body_section(
                self.inner_width,
                self.back_outer_width,
                self.radius
            );
        let side_segments = get_body_section(
                self.inner_width,
                self.outer_width,
                self.radius
            );

        let mut result = scad!(Union);
        //Position the front arms
        for i in 1..3
        {
            let rotated = scad!(Rotate(120. * i as f32, vec3(0.,0.,1.));
            {
                side_segments.clone()
            });

            result.add_child(rotated);
        }

        //ADd the back arm
        result.add_child(back_segment);

        result
    }

    /**
      Gets the outline of the block that goes at the back of the body
      for keeping the arm in place
    */
    fn get_back_mount_block(&self) -> ScadObject
    {
        //Percentage of the radius that the block should take up
        let length_factor = 0.5;

        let start_width = self.back_outer_width 
            + (self.inner_width - self.back_outer_width) 
            * length_factor;

        //Adding some padding to the arm width
        let arm_padding = 0.25;
        let total_arm_width = self.arm_width + arm_padding;

        let shape = {
            let points = vec!(
                    na::Vector2::new(self.radius * length_factor, total_arm_width/2.),
                    na::Vector2::new(self.radius * length_factor, start_width / 2.),
                    na::Vector2::new(self.radius, self.back_outer_width / 2.),
                    na::Vector2::new(self.radius, total_arm_width/2.),
                );
            scad!(Polygon(PolygonParameters::new(points)))
        };

        let mirrored = scad!(Mirror(vec3(0., 1., 0.));{shape.clone()});

        scad!(Union;{
            shape,
            mirrored
        })
    }

    /**
      Returns cylinders for the screwholes that should be on the front arms

      (The arm mount and arm blocker)
     */
    fn get_front_arm_screw_holes(&self) -> ScadObject
    {
        //The distance from the center to the point of the stopping screws
        let stopper_screw_distance = self.radius - 8.;

        //Distance from the center to the mount point of the arms
        let mount_screw_offset = self.radius - 25.;

        let mount_hole = scad!(Translate(vec3(mount_screw_offset, 0., 0.));{
            scad!(Cylinder(self.height, Diameter(SCREW_DIAMETER)))
        });

        let stopper_hole = scad!(Translate(vec3(stopper_screw_distance, self.arm_width / 2. + SCREW_DIAMETER / 2., 0.));{
            scad!(Cylinder(self.height, Diameter(SCREW_DIAMETER)))
        });

        let rotated = scad!(Rotate(120., vec3(0., 0., 1.));
        {
            mount_hole,
            stopper_hole
        });

        scad!(Union;
        {
            rotated.clone(),
            scad!(Mirror(vec3(0., 1., 0.));{rotated})
        })
    }

    /**
      Returns cylinders for the back screwholes
     */
    fn get_back_screwholes(&self) -> ScadObject
    {
        //The holes should be well in the mount block
        let x_offset = self.radius * 7. / 8.;

        scad!(Translate(vec3(x_offset, 0., 0.)); self.get_center_screwholes())
    }

    /**
      Returns screwholes centered around the x-axis with a radius that fits
      within the back block.
     */
    fn get_center_screwholes(&self) -> ScadObject
    {
        let y_offset = self.arm_width / 2. + SCREW_DIAMETER + 1.;

        let cylinders = scad!(Translate(vec3(0., y_offset, 0.));
        {
            scad!(Cylinder(self.get_bottom_total_height(), Diameter(SCREW_DIAMETER))),
        });

        scad!(Union;
        {
            cylinders.clone(),
            scad!(Mirror(vec3(0., 1., 0.)); cylinders)
        })
    }

    /**
      Returns the total height of the bottom object. The height of the bottom
      plate plus the height of the blocks.
     */
    fn get_bottom_total_height(&self) -> f32
    {
        self.arm_width + self.height
    }

    /**
        Returns the outline of the front section of the body
     */
    fn get_front_section(&self) -> ScadObject
    {
        let corner_radius = 2.;
        let length = self.front_section_length - corner_radius;
        let width = self.front_section_width - corner_radius * 2.;

        let points = vec!(
                na::Vector2::new(0., width / 2.),
                na::Vector2::new(0., -width / 2.),
                na::Vector2::new(-length, -width / 2.),
                na::Vector2::new(-length, width / 2.)
            );

        let polygon = scad!(Polygon(PolygonParameters::new(points)));
        let offset = scad!(Offset(OffsetType::Radius(corner_radius), false);{
            polygon
        });

        offset
    }

    /**
      Returns the outline of the part that is covered by the canopy

      This is the union of the front section and back arm mount
     */
    fn get_mid_section_outline(&self) -> ScadObject
    {
        let points = vec!(
            //Front
            na::Vector2::new(-self.front_section_length, self.front_section_width / 2.),
            na::Vector2::new(-self.front_section_length, -self.front_section_width / 2.),
            //Mid
            na::Vector2::new(0., self.inner_width / 2.),
            na::Vector2::new(0., -self.inner_width / 2.),
            //Back
            na::Vector2::new(self.radius, self.back_outer_width / 2.),
            na::Vector2::new(self.radius, -self.back_outer_width / 2.)
        );

        let path = vec!(
                vec!(0, 2, 4, 5, 3, 1)
            );

        scad!(Polygon(PolygonParameters::new(points).multi_vector_path(path)))
    }

    /**
      Returns an outline around the mid_section
     */
    fn get_top_plate_canopy_edges(&self) -> ScadObject
    {
        //Function for doing the linear extrusion
        fn extrusion_function(outline: ScadObject, height: f32) -> ScadObject
        {
            let linear_extrude_parameters = LinExtrudeParams{
                height: height,
                ..Default::default()
            };

            scad!(LinearExtrude(linear_extrude_parameters);
            {
                outline
            })
        };


        let outer_radius_offset =
                -self.canopy_thickness
                + self.edge_thickness
                - self.edge_padding;

        let inner_radius_offset = -self.canopy_thickness;

        let outer = {
            let polygon = scad!(Offset(OffsetType::Radius(outer_radius_offset), false);
            {
                self.get_mid_section_outline()
            });

            extrusion_function(polygon, self.edge_height)
        };
        let inner = {
            let polygon = scad!(Offset(OffsetType::Radius(inner_radius_offset), false);
            {
                self.get_mid_section_outline()
            });

            extrusion_function(polygon, self.edge_height)
        };

        scad!(Difference;{
            outer,
            inner
        })
    }

    /**
      Returns the cutout for the camera hole that goes in the top part
      of the frame
    */
    fn get_camera_box_top_cutout(&self) -> ScadObject
    {
        let x_end = -(self.front_section_length - self.canopy_thickness);
        let x_start = x_end + self.camera_box_length;
        let width = self.front_section_width - self.canopy_thickness * 2.;

        let points = vec!(
                na::Vector2::new(x_start, width / 2.),
                na::Vector2::new(x_end, width / 2.),
                na::Vector2::new(x_end, -width / 2.),
                na::Vector2::new(x_start, -width / 2.),
            );

        scad!(Polygon(PolygonParameters::new(points)))
    }

    /**
      Returns the outline of a block that makes up the bulk of the camera 
      box that goes on the bottom frame piece
     */
    fn get_camera_box_outline(&self) -> ScadObject
    {
        let corner_radius = 0.;
        let x_start = -(
                self.front_section_length 
                - self.camera_box_length
                - self.canopy_thickness * 2.
            );

        let x_end = -(self.front_section_length - corner_radius);
        let y_start = self.front_section_width / 2. - corner_radius;
        let y_end = -y_start;

        let points = vec!(
                na::Vector2::new(x_start, y_start),
                na::Vector2::new(x_end, y_start),
                na::Vector2::new(x_end, y_end),
                na::Vector2::new(x_start, y_end),
            );

        let polygon = scad!(Polygon(PolygonParameters::new(points)));

        scad!(Offset(OffsetType::Radius(corner_radius), false); polygon)
    }

    /**
      Returns an outline of the edge that fits inside the camera_box_top_cutout
    */
    fn get_camera_box_bottom_edge_outline(&self) -> ScadObject
    {
        let offset = -(self.canopy_thickness + self.camera_box_edge_padding);
        scad!(Offset(OffsetType::Delta(offset), false);{
            self.get_camera_box_outline()
        })
    }
    /**
      Returns a 2d outline of the hole part of the camera box
     */
    fn get_camera_box_bottom_cutout_outline(&self) -> ScadObject
    {
        let offset = -self.camera_box_edge_width;

        scad!(Offset(OffsetType::Delta(offset), false);{
            self.get_camera_box_bottom_edge_outline()
        })
    }

    /**
      Translate an object to the location of the hole where the motor wires
      come out.

      This is the small hole that goes through the back block
     */
    fn place_object_at_motor_wire_hole(&self, object: ScadObject) -> ScadObject
    {
        let padding = 1.5;
        let hole_x = self.radius * 3. / 4.;
        let hole_y = self.arm_width / 2. + self.motor_wire_hole_radius + padding;

        scad!(Translate(vec3(hole_x, hole_y, 0.)); object)
    }

    /**
      Returns the cutout of the motor_wire_hole that goes in the top part of the
      frame. Includes space for water tight seal
    */
    fn get_top_plate_motor_wire_hole(&self) -> ScadObject
    {
        let hole = scad!(Cylinder(self.height, Radius(self.motor_wire_hole_radius)));

        let o_ring_diameter = self.motor_wire_hole_radius + 2.;
        let o_ring_height =1.;
        let o_ring_hole = scad!(Cylinder(o_ring_height, Radius(o_ring_diameter)));

        scad!(Union;
        {
            self.place_object_at_motor_wire_hole(hole),
            self.place_object_at_motor_wire_hole(o_ring_hole)
        })
    }

    /**
      Returns a hole where the battery wires can go through in front of the
      flight controller
     */
    fn get_battery_wire_hole(&self) -> ScadObject
    {
        let radius = 3.;
        let position = vec3(-self.radius / 3., 0., 0.);
        
        let hole = scad!(Cylinder(self.height, Radius(radius)));

        scad!(Translate(position); hole)
    }

    /**
      Returns a cylinder that cuts a hole for the camera
     */
    fn get_camera_lens_hole(&self, z_offset: f32) -> ScadObject
    {
        let camera_board = BoardCamera::new();
        let offset = -(self.front_section_length - camera_board.lens_length / 2.);

        let hole = camera_board.get_lens_hole();
        let rotated = scad!(Rotate(-90., vec3(0., 1., 0.)); hole);

        scad!(Translate(vec3(offset, 0., z_offset)); rotated)
    }

    /**
      Returns a box that makes sure the sides of the frame don't exceed 14cm
      to fit inside the printer without rotating.
     */
    fn get_side_bounds(&self) -> ScadObject
    {
        let width = 140.;

        centered_cube(vec3(10000., width, 1000.), (true, true, false))
    }
}


use std::io::prelude::*;
use std::fs::OpenOptions;
fn add_text_to_history_file(content: &str, history_file: &str)
{
    let mut target_file = OpenOptions::new()
        .append(true)
        .create(true)
        .open(history_file).unwrap();

    let file_divider = "##__start_of_new_file__##\n";

    target_file.write_all(file_divider.as_bytes()).unwrap();
    target_file.write_all(content.as_bytes()).unwrap();
}


fn test_esc_stack(sfile: &mut ScadFile)
{
    let esc_rotation = 90.;
    let esc = add_named_color(
        "crimson",
        scad!(Rotate(esc_rotation, vec3(0., 0., 1.)); {
            Esc::new().get_pcb((true, true, false)),
        })
    );
    let holder = add_named_color(
        "SaddleBrown",
        scad!(Rotate(esc_rotation, vec3(0., 0., 1.)); {
            EscStack::new().get_mid_section()
        })
    );

    let x_pos = 40.;
    let z_offset = 6.;
    for i in 0..3
    {
        sfile.add_object(
                scad!(Translate(vec3(x_pos, 0., 30. + (i as f32) * z_offset)); esc.clone())
            );
        sfile.add_object(
                scad!(Translate(vec3(x_pos, 0., 30. + (i as f32) * z_offset + 3.)); holder.clone())
            );
    }
}


fn main() 
{
    let mut sfile = ScadFile::new();
    sfile.set_detail(20);

    sfile.add_object(TricopterBody::new().get_body_bottom());
    sfile.add_object(scad!(Translate(vec3(0., 0., 20.)); TricopterBody::new().get_body_top()));
    //sfile.add_object(
    //        add_named_color(
    //            "steelblue",
    //            scad!(Translate(vec3(0., 0., 30.)); NazeBoard::new().get_board())
    //        )
    //    );
    //sfile.add_object(
    //        add_named_color(
    //            "dimgray",
    //            scad!(Translate(vec3(-40., 0., 30.)); 
    //            {
    //                scad!(Rotate(-90., vec3(0., 1., 0.));
    //                {
    //                    BoardCamera::new().get_model(),
    //                })
    //            })
    //        )
    //    );

    //test_esc_stack(&mut sfile);


    sfile.write_to_file(String::from("out.scad"));

    add_text_to_history_file(&sfile.get_code(), "frame_history.scad");
}
