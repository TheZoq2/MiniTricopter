#![allow(dead_code)]
#[macro_use]
extern crate scad;
extern crate scad_util;
extern crate nalgebra as na;

use scad::*;

use std::string::String;

use scad_util::{
    add_named_color,
    constants::{z_axis, x_axis, y_axis},
    nut
};

const SCREW_DIAMETER: f32 = 3.5;

fn get_vtx_mount() -> ScadObject
{
    let hole_diameter = 3.5;
    let hole_padding = 2.;

    let hole_edge_distance = 8.;
    let vtx_thickness = 8.;
    let vtx_width = 21.;

    let put_at_hole = |object: ScadObject|
    {
        scad!(Union;
        {
            scad!(Translate2d(vec2(vtx_width/2. + (hole_edge_distance+vtx_thickness), 0.));
            {
                object.clone()
            }),
            scad!(Translate2d(vec2(-(vtx_width/2. + (hole_edge_distance+vtx_thickness)), 0.));
            {
                object.clone()
            }),
        })
    };

    let outer_holes = put_at_hole(scad!(Circle(Diameter(hole_diameter + hole_padding*2.))));

    let center_box = centered_square(vec2(vtx_width, vtx_width), (true, true));

    let outer_shape = scad!(Hull;
    {
        outer_holes,
        center_box
    });

    let center_cutout = scad!(Scale2d(vec2(0.3, 0.3)); outer_shape.clone());

    let screwhole = put_at_hole(scad!(Circle(Diameter(hole_diameter))));

    let shape = scad!(Union;
    {
        scad!(Difference;
        {
            outer_shape,
            screwhole,
            center_cutout,
        }),
        centered_square(vec2(3., vtx_width), (true,true))
    });

    scad!(LinearExtrude(LinExtrudeParams{height: 1.5, .. Default::default()}); shape)
}

fn get_m3_screw(length: f32) -> ScadObject
{
    let screw_padding = 0.5;
    let head_padding = 1.;
    let diameter = 3. + screw_padding;
    let head_diameter = 6. + head_padding;
    let head_height = 2.;

    scad!(Union;
    {
        scad!(Cylinder(head_height, Diameter(head_diameter))),
        scad!(Cylinder(length + head_height, Diameter(diameter))),
    })
}

qstruct!(NazeBoard()
{
    hole_diameter: f32 = 3.,
    hole_distance: f32 = 30.5,
    hole_padding_radius: f32 = hole_diameter / 2. + 1.,
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

    fn get_width(&self) -> f32
    {
        self.hole_distance + self.hole_padding_radius*2.
    }
}

qstruct!(DysEsc()
{
    x_length: f32 = 40.,
    y_length: f32 = 43.3,

    extension_width: f32 = 30.
});

impl DysEsc
{
    fn get_board(&self) -> ScadObject
    {
        let height = 3.5;

        let naze = NazeBoard::new();

        let main = naze.get_board();

        let extensions = scad!(Union;
            {
                centered_cube(vec3(self.extension_width, self.y_length, height), (true, true, false)),
                centered_cube(vec3(self.x_length, self.extension_width, height), (true, true, false))
            });

        let union = scad!(Union;
        {
            main,
            extensions
        });

        scad!(Difference;
        {
            union,
            naze.get_holes(height)
        })
    }
}

qstruct!(BoardCamera()
{
    width: f32 = 32.,
    thickness: f32 = 4.,
    lens_diameter: f32 = 17.,
    lens_length: f32 = 24.,
    snowproof_padding_radus: f32 = 3.,
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
        let snowproofing_padding_radius = self.snowproof_padding_radus;

        let total_radius = snowproofing_padding_radius + self.lens_diameter / 2.;

        scad!(Cylinder(self.lens_length, Radius(total_radius)))
    }
}

fn get_camera_water_seal(camera: &BoardCamera, tricopter_body: &TricopterBody) -> ScadObject
{
    let inner_radius = camera.lens_diameter / 2.;
    let outer_radius = camera.snowproof_padding_radus + inner_radius;
    let thickness = tricopter_body.canopy_thickness;
    let outer_thickness = thickness + 5.;
    let outer_shell_radius = outer_radius + 5.;

    let shell_square = {
        let size = vec2(outer_shell_radius - inner_radius, outer_thickness);
        let centering = (false, true);
        centered_square(size, centering)
    };

    let outer_cutoff = {
        let cube = centered_square(vec2(outer_radius, thickness), (false, true));

        scad!(Translate2d(vec2(outer_radius - inner_radius, 0.)); cube)
    };

    let outline = scad!(Difference;
    {
        shell_square,
        outer_cutoff
    });

    let translated_outline = scad!(Translate2d(vec2(inner_radius, 0.));
    {
        outline
    });

    scad!(RotateExtrude(Default::default()); 
    {
        translated_outline
    })
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
    layer_thickness: f32 = 2.,
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
                ),
            centered_cube(
                vec3(self.esc.width, self.esc.length, self.layer_thickness), 
                (true, true, false)
            ),
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

fn get_cable_tie_hole(height: f32, z_rotation: f32) -> ScadObject
{
    let separation = 4.;
    let length = 4.;
    let hole_width = 3.;

    let cutout = scad!(Cube(vec3(hole_width, length, height)));
    let translated = scad!(Translate(vec3(separation/2., 0., 0.)); cutout);

    scad!(Rotate(z_rotation, vec3(0., 0., 1.));
    {
        translated.clone(),
        scad!(Mirror(vec3(1., 0., 0.)); translated.clone()),
    })
}

qstruct!(TricopterBody()
{
    radius: f32 = 80.,
    top_height: f32 = 5.,
    height: f32 = 4.,
    outer_width: f32 = 23.,
    back_outer_width: f32 = 33.,
    inner_width: f32 = 50.,
    center_width: f32 = 56.,

    back_block_length_factor: f32 = 0.5,

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

    mounting_screw_outline_radius: f32 = 3.,

    canopy_max_height: f32 = 31.,
    screw_mount_height: f32 = 7.,
    canopy_bottom_min_height: f32 = screw_mount_height,
    screwhead_diameter: f32 = 6.5,

    side_plate_arc_width: f32 = 36.,
    side_plate_arc_height: f32 = 19.,
    side_plate_thickness: f32 = 2.,
    side_plate_mount_length: f32 = 15.,
    side_plate_front_screw_top_offset: f32 = 7.,
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
                self.get_front_screw_tab_outline(),
                self.get_back_screw_tab_outline(),
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
        let with_holes = scad!(Difference;
        {
            body
            , self.get_front_arm_screw_holes()
            , self.get_back_screwholes()
            , camera_box_cutout
            , camera_lens_hole
            , self.get_battery_strap_holes()
            , self.place_object_at_motor_wire_holes(
                        self.get_back_block_cable_hole())
            , self.get_cable_tie_holes()
        });

        scad!(Intersection;
        {
            with_holes,
            self.get_side_bounds()
        })
    }

    /**
      Main function for the top section of the body
    */
    pub fn get_body_top(&self, filter_front: bool) -> ScadObject
    {
        let front_back_spliter = {
            let main_cube = centered_cube(vec3(100., 100., 100.), (false, true, false));
            let lower_cube = scad!(Translate(vec3(3., 0., 0.)); {
                centered_cube(
                    vec3(100., 100., self.top_height / 2.),
                    (false, true, false)
                )});

            self.place_object_at_motor_wire_holes(scad!(Union; {
                scad!(Translate(vec3(0., 0., self.top_height / 2.)); {
                    main_cube,
                }),
                lower_cube
            }))
        };


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
                self.get_mid_section_outline(),
                self.get_front_section(),
                self.get_front_screw_tab_outline(),
                self.get_back_screw_tab_outline(),
            }),

            scad!(Translate(vec3(0., 0., self.height));{
                self.get_top_plate_canopy_edges()
            }),
        });

        let camera_box = scad!(LinearExtrude(linear_extrude.clone());
            {
                self.get_camera_box_top_cutout()
            });

        let screwholes = scad!(Union;
        {
            NazeBoard::new().place_object_at_holes(get_m3_screw(self.height)),
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

        let full_shape = scad!(Intersection;{
            with_holes,
            self.get_side_bounds()
        });

        if filter_front {
            scad!(Difference; {
                full_shape,
                front_back_spliter
            })
        }
        else {
            scad!(Intersection; {
                full_shape,
                front_back_spliter
            })
        }
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
        let length_factor = self.back_block_length_factor;

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

        let cylinder = scad!(Cylinder(self.get_bottom_total_height(), Diameter(SCREW_DIAMETER)));
        let cylinders = scad!(Translate(vec3(0., y_offset, 0.));
        {
            cylinder.clone()
        });

        scad!(Union;
        {
            cylinders.clone(),
            scad!(Mirror(vec3(0., 1., 0.)); cylinders),
            cylinder
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
        let corner_radius = self.front_section_corner_radius;
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
            na::Vector2::new(0., self.center_width / 2.),
            na::Vector2::new(0., -self.center_width / 2.),
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
                - self.canopy_thickness
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
        let corner_radius = self.front_section_corner_radius;
        let x_start = -(
                self.front_section_length 
                - self.camera_box_length
                - self.canopy_thickness * 2.
                + corner_radius
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
    fn place_object_at_motor_wire_holes(&self, object: ScadObject) -> ScadObject
    {
        let padding = 1.5;
        let hole_x = self.radius * 3. / 4.;
        let hole_y = self.arm_width / 2. + self.motor_wire_hole_radius + padding;

        let translated = scad!(Translate(vec3(hole_x, hole_y, 0.)); object);

        let mirrored = scad!(Mirror(vec3(0., 1., 0.));
        {
            translated.clone()
        });

        scad!(Union;
        {
            translated,
            mirrored
        })
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
            self.place_object_at_motor_wire_holes(hole),
            self.place_object_at_motor_wire_holes(o_ring_hole)
        })
    }

    /**
      Returns a hole where the battery wires can go through in front of the
      flight controller
     */
    fn get_battery_wire_hole(&self) -> ScadObject
    {
        let radius = 4.;
        let position = vec3(-self.radius / 3., 0., 0.);
        
        let hole = scad!(Cylinder(self.height, Radius(radius)));

        scad!(Translate(position); hole)
    }

    /**
      Returns a cylinder that cuts a hole for the camera

      z_offset is the offset in the z axis from the bottom of the top part of
      the frame
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

    /**
      Returns cubes where the battery straps should go
    */
    fn get_battery_strap_holes(&self) -> ScadObject
    {
        let y_separation = 17.;
        let x_size = 27.;
        let y_size = 4.;

        let cube = 
            centered_cube(vec3(x_size, y_size, self.height), (true,true,false));

        let translated = scad!(Translate(vec3(0., y_separation, 0.)); cube);

        scad!(Union;
        {
            translated.clone(),
            scad!(Mirror(vec3(0., 1., 0.)); translated)
        })
    }

    /**
      Positions an object at the location of the 2 mounting screw holes in the
      front
    */
    fn place_object_at_front_mount_locations(&self, object: ScadObject)
            -> ScadObject
    {
        let offset_from_side = self.mounting_screw_outline_radius;
        let x_offset = -(self.front_section_length * 11. / 16.);
        let y_separation = self.front_section_width / 2. + offset_from_side;

        scad!(Translate(vec3(x_offset, 0., 0.));
        {
            scad!(Translate(vec3(0., y_separation, 0.)); object.clone()),
            scad!(Translate(vec3(0., -y_separation, 0.)); object.clone()),
        })
    }

    /**
      Returns an outline of the screw holes that connect the top canopy with
      the bottom section
     */
    fn get_front_screw_tab_outline(&self) -> ScadObject
    {
        let outline_radius = self.mounting_screw_outline_radius;
        let outline_circle = scad!(Circle(Radius(outline_radius)));
        let outline_circles = 
            self.place_object_at_front_mount_locations(outline_circle);

        scad!(Hull;
        {
            outline_circles
        })
    }

    fn get_front_screwholes(&self, size: CircleType) -> ScadObject {
        let screwhole = scad!(Circle(size));
        let screwholes = self.place_object_at_front_mount_locations(screwhole);

        screwholes
    }

    /**
      Returns the outline of the mounting screws for the back section
    */
    fn get_back_screw_tab_outline(&self) -> ScadObject
    {
        let outline_radius = self.mounting_screw_outline_radius;

        let outline_circle = scad!(Circle(Radius(outline_radius)));

        let x_start = self.radius - outline_radius;
        let x_end = self.radius + outline_radius;

        let hull = scad!(Hull;{
            scad!(Translate2d(vec2(x_start, 0.)); outline_circle.clone()),
            scad!(Translate2d(vec2(x_end, 0.)); outline_circle.clone()),
        });

        scad!(Difference;
        {
            self.place_object_at_back_screwholes(hull),
        })
    }

    fn circle_at_back_screwholes(&self, size: CircleType) -> ScadObject {
        let outline_radius = self.mounting_screw_outline_radius;

        let x_end = self.radius + outline_radius;

        let hole = {
            let circle = scad!(Circle(size));

            scad!(Translate2d(vec2(x_end, 0.)); circle)
        };

        scad!(Difference;
        {
            self.place_object_at_back_screwholes(hole)
        })
    }

    fn place_object_at_back_screwholes(&self, object: ScadObject) -> ScadObject {
        let y_separation = self.outer_width * 7. / 8.;

        scad!(Union;
        {
            scad!(Translate2d(vec2(0., y_separation / 2.)); object.clone()),
            scad!(Translate2d(vec2(0., -y_separation / 2.)); object)
        })
    }


    /**
      Returns the hole for routing cables thorugh the back section of the bottom
      body section
    */
    fn get_back_block_cable_hole(&self) -> ScadObject
    {
        let height = self.get_bottom_total_height() * 3. / 8.;
        let radius = Radius(self.motor_wire_hole_radius);
        let bottom_length = self.radius * self.back_block_length_factor / 2.;

        let top_cylinder = 
            scad!(Cylinder(height, radius.clone()));

        let middle_sphere = scad!(Sphere(radius.clone()));

        let horizontal_cylinder = {
            let cylinder = scad!(Cylinder(bottom_length, radius.clone()));
            scad!(Rotate(-90., vec3(0., 1., 0.)); cylinder)
        };

        let z_position = self.get_bottom_total_height() - height;
        scad!(Translate(vec3(0., 0., z_position));
        {
            top_cylinder,
            middle_sphere,
            horizontal_cylinder
        })
    }

    /**
      Returns holes for attaching cable ties to the bottom section of the body
     */
    fn get_cable_tie_holes(&self) -> ScadObject
    {
        //Location of the front zip ties
        let front_y_separation = self.inner_width;
        let front_x_offset = -(self.front_section_length / 3.);

        //Location of the back zip tie
        let back_x = self.radius / 3.;
        let back_y = self.outer_width / 2.;

        let points = vec!(
                na::Vector3::new(front_x_offset, front_y_separation / 2., 0.)
                , na::Vector3::new(front_x_offset, -front_y_separation / 2., 0.)
                , na::Vector3::new(back_x, back_y, 0.)
            );

        let mut result = scad!(Union);

        for point in points
        {
            let object = scad!(Translate(point); 
            {
                get_cable_tie_hole(self.height, 90.)
            });

            result.add_child(object);
        }

        result
    }

    /**
      Returns the outline of the canopy in the x-z plane
    */
    fn get_canopy_xz_outline(&self, additional_height: f32) -> ScadObject
    {
        let bottom_offset = additional_height;

        let max_height = self.canopy_max_height;
        
        let front_offset = 10.;

        let points = vec!(
            vec2(-self.front_section_length, bottom_offset)
            , vec2(-self.front_section_length, max_height - front_offset)
            , vec2(-self.front_section_length + front_offset, max_height)
            , vec2(self.radius * 2. / 8., max_height)
            , vec2(self.radius, self.canopy_bottom_min_height)
            , vec2(self.radius, bottom_offset)
        );

        scad!(Polygon(PolygonParameters::new(points)))
    }

    fn get_canopy_yz_outline(&self, additional_height: f32) -> ScadObject
    {
        let bottom_offset = additional_height;

        let top_width = self.inner_width * 7. / 8.;

        let triangle_height_above_canopy = top_width / 2.;

        let triangle_height = triangle_height_above_canopy
                              + self.canopy_max_height;
                              - bottom_offset;

        let points = vec!(
                na::Vector2::new(triangle_height, bottom_offset),
                na::Vector2::new(-triangle_height, bottom_offset),
                na::Vector2::new(0., triangle_height)
            );

        scad!(Polygon(PolygonParameters::new(points)))
    }

    /**
      Returns the outside of the canopy
    */
    fn get_canopy_outside(&self, xz_yz_offset:f32) -> ScadObject
    {
        let offset_function = |object| {
            scad!(Offset(OffsetType::Delta(xz_yz_offset), true); object)
        };

        let xy_outline = offset_function(self.get_mid_section_outline());
        let xz_outline = offset_function(self.get_canopy_xz_outline(xz_yz_offset));
        let yz_outline = offset_function(self.get_canopy_yz_outline(xz_yz_offset));

        let extrude_params = LinExtrudeParams
        {
            height:500.,
            center: true,
            ..Default::default()
        };

        let extruded_xy = 
            scad!(LinearExtrude(extrude_params.clone()); xy_outline);

        let extruded_xz = {
            let extruded =
                scad!(LinearExtrude(extrude_params.clone()); xz_outline);

            scad!(Rotate(90., vec3(1., 0., 0.)); extruded)
        };
        let extruded_yz = {
            let extruded = 
                scad!(LinearExtrude(extrude_params); yz_outline);

            let initial_rotation = scad!(Rotate(90., vec3(1., 0., 0.)); extruded);

            scad!(Rotate(90., vec3(0., 0., 1.)); initial_rotation)
        };

        let canopy_front_roundoff = {
            let outline = {
                let roundoff_amount = 10.;
                let front_length = self.front_section_length;
                let points = vec!(
                    vec2(200., self.outer_width),
                    vec2(-front_length + roundoff_amount, self.back_outer_width),
                    vec2(-front_length, self.outer_width - roundoff_amount),
                    vec2(-front_length, -self.outer_width + roundoff_amount),
                    vec2(-front_length + roundoff_amount, -self.back_outer_width),
                    vec2(200., -self.outer_width)
                );

                scad!(Polygon(PolygonParameters::new(points)))
            };

            let extrude_params = LinExtrudeParams
            {
                height:500.,
                center: true,
                ..Default::default()
            };

            let extruded = scad!(LinearExtrude(extrude_params); {
                offset_function(outline)
            });

            let shallow = {
                let rotated = scad!(Rotate(30., vec3(0., 1., 0.)); extruded.clone());
                scad!(Translate(vec3(0., 0., 7.)); rotated)
            };
            let steep = {
                let rotated = scad!(Rotate(60., vec3(0., 1., 0.)); extruded);
                scad!(Translate(vec3(0., 0., -9.)); rotated)
            };
            scad!(Intersection; {
                shallow,
                steep
            })
        };

        scad!(Intersection;
        {
            extruded_xy,
            extruded_xz,
            extruded_yz,
            canopy_front_roundoff
        })
    }

    fn extrude_canopy_edge(&self, object: ScadObject, extra_height: f32) -> ScadObject
    {
        let height = self.edge_height;

        let extrude_params = LinExtrudeParams
                 {
                     height: height + extra_height,
                     .. Default::default()
                 };

        let extruded = scad!(LinearExtrude(extrude_params); object);

        scad!(Translate(vec3(0., 0., -height)); extruded)
    }

    fn canopy_edge_cutout(&self) -> ScadObject
    {
        let offset = -(self.edge_thickness - self.edge_padding);

        scad!(Offset(OffsetType::Delta(offset), false);
        {
            self.get_mid_section_outline()
        })
    }

    fn get_canopy_screw_tabs(&self, height: f32) -> ScadObject
    {
        let extrude_params = LinExtrudeParams{
            height: height
            , .. Default::default()
        };

        scad!(Translate(vec3(0., 0., -self.edge_height));
        {
            scad!(LinearExtrude(extrude_params.clone()); 
                  self.get_front_screw_tab_outline()),
            scad!(LinearExtrude(extrude_params); 
                  self.get_back_screw_tab_outline()),
        })
    }

    fn get_canopy_screwholes(&self, screw_length: f32) -> ScadObject {
        let screwhole_extrude_params = LinExtrudeParams{
            height: screw_length
            , .. Default::default()
        };
        let screwhead_extrude_params = LinExtrudeParams{
            height: self.canopy_max_height
            , .. Default::default()
        };

        let head_holes = scad!(LinearExtrude(screwhead_extrude_params); {
            self.get_front_screwholes(Diameter(self.screwhead_diameter)),
            self.circle_at_back_screwholes(Diameter(self.screwhead_diameter))
        });

        let holes = scad!(LinearExtrude(screwhole_extrude_params); {
            self.get_front_screwholes(Diameter(SCREW_DIAMETER)),
            self.circle_at_back_screwholes(Diameter(SCREW_DIAMETER))
        });
        scad!(Union; {
            scad!(Translate(vec3(0., 0., -self.edge_height)); holes),
            scad!(Translate(vec3(0., 0., screw_length - self.edge_height)); {
                head_holes
            })
        })
    }

    fn get_front_fillet(&self, height: f32) -> ScadObject
    {
        let radius = self.front_section_corner_radius;

        let centering = (false, true);
        let main_size = vec2(self.radius, self.front_section_width);

        let main_square = centered_square(main_size, centering);
        let small_square = scad!(Offset(OffsetType::Delta(-radius), false);
                    main_square.clone());
        let rounded_square = scad!(Offset(OffsetType::Radius(radius), false);
                    small_square);

        let corners = scad!(Difference;
        {
            main_square.clone(),
            rounded_square
        });

        let cutoff_square = scad!(Translate2d(vec2(self.radius / 2., 0.));
        {
            centered_square(vec2(self.radius/2., self.front_section_width),
                    centering)
        });

        let outline = scad!(Difference;
        {
            scad!(Intersection;
            {
                main_square
                , corners
            })
            , cutoff_square
        });

        let translated = scad!(Translate2d(vec2(-self.front_section_length, 0.));
        {
            outline
        });

        let extrude_params = LinExtrudeParams{
            height: height * 2.,
            center: true,
            .. Default::default()
        };

        scad!(LinearExtrude(extrude_params); translated)
    }

    fn get_vtx_connector_hole(&self) -> ScadObject
    {
        let screw_section_diameter = 6.;
        let screw_section_length = 2.5;
        let outer_diameter = 8.;
        let outer_length = self.canopy_thickness - screw_section_length + 5.;

        let z_offset = self.canopy_max_height / 2.;

        let angle = ((self.inner_width - self.back_outer_width) / 2.)
                    .atan2(self.radius)
                    .to_degrees();

        //The cylinders that make up the hole
        let small_cylinder = scad!(Cylinder(screw_section_length + 10., 
                           Diameter(screw_section_diameter)));
        let big_cylinder = 
                    scad!(Cylinder(outer_length, Diameter(outer_diameter)));

        let cutout = scad!(Union;
        {
            scad!(Translate(vec3(0., 0., -outer_length));
            {
                big_cylinder
            }),
            small_cylinder
        });

        //Putting the top at 0
        let translated = scad!(Translate(vec3(0., 0., -screw_section_length));
                               cutout);

        //Moving the object along the back wall
        let moved = scad!(Translate(vec3(-self.radius / 6., 0., 0.));
                          translated);

        let rotated = scad!(Rotate(angle, vec3(0., 0., 1.));
        {
            scad!(Rotate(90., vec3(1., 0., 0.)); moved)
        });


        //Positioning the object at the corner of the back part
        let corner_pos = vec3(self.radius, -self.back_outer_width / 2., z_offset);

        scad!(Translate(corner_pos);rotated)
    }

    fn get_canopy(&self) -> ScadObject
    {
        let extra_offset = 1.;
        let canopy_edge = scad!(Offset(OffsetType::Radius(extra_offset), false); {
            self.get_mid_section_outline()
        });

        let body = scad!(Union;
        {
            self.get_canopy_outside(extra_offset)
            , self.get_canopy_screw_tabs(self.screw_mount_height)
            , self.extrude_canopy_edge(canopy_edge, 3.)
        });

        let camera_offset = self.edge_height;
        scad!(Difference;
        {
            body
            , self.get_canopy_outside(-3.)
            , self.get_camera_lens_hole(camera_offset)
            , self.extrude_canopy_edge(self.canopy_edge_cutout(), 0.)
            , self.get_canopy_screwholes(self.screw_mount_height)
            // , self.get_front_fillet(self.canopy_max_height)
        })
    }

    fn get_side_plate_mount(&self) -> ScadObject {
        // The finnished part is bent outwards indicating that the inner shape is
        // too small, this is compensation for that.
        let padding = 4.;

        let thickness = self.side_plate_thickness;
        let base_height = self.top_height + thickness * 2.;
        let length = self.side_plate_mount_length;
        let arch_width = self.side_plate_arc_width;
        let arch_height = self.side_plate_arc_height;
        let base_outer_shape = {
            let base = self.get_body_shape();

            scad!(Offset(OffsetType::Delta(thickness + padding/2.), false); base)
        };

        let base_extruded = scad!(LinearExtrude(
            LinExtrudeParams {height: base_height, .. Default::default()}
        ); base_outer_shape);

        let inner_cutout = {
            let base = self.get_body_shape();

            let offset = scad!(Offset(OffsetType::Delta(-5.), false); base);

            scad!(LinearExtrude(LinExtrudeParams {
                height: base_height-thickness,
                .. Default::default()
            }); offset)
        };

        let inner_inner_cutout = {
            let base = self.get_body_shape();

            let offset = scad!(Offset(OffsetType::Delta(padding/2.), false); base);

            let shape = scad!(LinearExtrude(LinExtrudeParams {
                height: self.top_height,
                .. Default::default()
            }); offset);

            scad!(Translate(vec3(0., 0., thickness)); {
                shape
            })
        };

        let back_cutout = {
            let shape = centered_cube(vec3(100., 100., 100.), (false, true, true));

            scad!(Translate(x_axis()*self.radius*self.back_block_length_factor); {
                shape
            })
        };

        let front_cutout = {
            let shape = centered_cube(vec3(500., 500., 500.), (false, true, true));

            let x_trans = self.radius * self.back_block_length_factor-length;
            scad!(Translate(x_axis() * x_trans); {
                scad!(Mirror(x_axis()); shape)
            })
        };

        let arch_outer = {
            let height = base_height + arch_height + thickness;
            let width = arch_width;
            centered_cube(vec3(100., width, height), (false, true, false))
        };
        let arch_inner = {
            let height = arch_height + base_height - thickness * 2.;
            let width = arch_width - thickness * 2.;
            let shape = centered_cube(
                vec3(100., width, height),
                (false, true, false)
            );
            scad!(Translate(z_axis() * thickness * 2.); {shape})
        };

        scad!(Difference; {
            scad!(Union; {
                base_extruded,
                arch_outer
            }),
            inner_cutout,
            inner_inner_cutout,
            back_cutout,
            front_cutout,
            arch_inner
        })
    }

    fn side_plate_shape(&self) -> ScadObject {
        let back_length = self.radius * self.back_block_length_factor;
        let back_height = self.side_plate_arc_height;
        let thickness = self.side_plate_thickness;
        let arch_length = self.side_plate_mount_length;

        let center_height = 27.;
        let center_length = 36.;
        let front_height = 20.;
        let front_length = 45.;
        let esc_height = 6.;
        let groove_depth = 1.5;
        let screw_diameter = 3.2;

        let shape = Polygon(PolygonParameters::new(vec!{
            vec2(-back_length, thickness),
            vec2(-back_length, back_height + thickness),
            vec2(-center_length / 2., center_height),
            vec2(center_length / 2., center_height),
            vec2(front_length, front_height),
            vec2(front_length, 0.),
            vec2(center_length/2., 0.),
            vec2(center_length/2., esc_height),
            vec2(-center_length/2., esc_height),
            vec2(-center_length/2., 0.),
            vec2(-(back_length - arch_length), 0.),
            vec2(-(back_length - arch_length), thickness),
        }));

        let holes = {
            let hole_shape = scad!(Circle(Diameter(screw_diameter)));

            // Screwholes
            let back_x_offset = back_length - arch_length / 2.;
            let back_low_pos = vec2(-back_x_offset, back_height/3.);
            let back_high_pos = vec2(-back_x_offset, 3.*back_height/4.);
            let back_low = scad!(Translate2d(back_low_pos); hole_shape.clone());
            let back_high = scad!(Translate2d(back_high_pos); hole_shape.clone());

            let fc_hole = {
                let y_offset = 14.;
                let height = 6.;
                let hole = centered_square(
                    vec2(center_length, height),
                    (true, true)
                );
                scad!(Translate2d(vec2(0., y_offset)); hole)
            };

            scad!(Union; {
                back_low,
                back_high,
                fc_hole
            })
        };

        let groove_outline = {
            let y_offset = 14.;
            let width = 7.;
            let x_offset = center_length - width/2.;

            let shape = centered_square(vec2(width, 100.), (true, false));
            let back = scad!(Translate2d(vec2(-x_offset / 2., y_offset)); {
                shape.clone()
            });
            let front = scad!(Translate2d(vec2(x_offset / 2., y_offset)); {
                shape
            });

            scad!(Union; {
                back,
                front
            })
        };

        let (front_nut_cut, front_nut_outer) = {
            let x_offset = center_length / 2. + (front_length - center_length /2.) / 2.;
            let y_padding = self.side_plate_front_screw_top_offset;
            let y_offset = front_height
                + (center_height - front_height) / 2.
                - y_padding;
            let extra_height = 2.;
            let inner_nut_offset = 1.5;
            let inner_nut_width = 5.3;
            let outer_nut_width = inner_nut_width + 5.;

            let hole = scad!(Cylinder(100., Diameter(screw_diameter)));
            let nut_outer = {
                let shape = nut(outer_nut_width, extra_height);
                scad!(Translate(vec3(0., 0., thickness)); shape)
            };

            let nut_inner = {
                let shape = nut(inner_nut_width, 100.);
                scad!(Translate(vec3(0., 0., inner_nut_offset)); shape)
            };

            let position = |obj| {
                scad!(Mirror(z_axis()); {
                    scad!(Translate(vec3(x_offset, y_offset, -thickness)); {obj})
                })
            };

            let cutout = scad!(Union; {
                nut_inner,
                hole
            });

            (position(cutout), position(nut_outer))
        };

        let outer_extruded = {
            let shape = scad!(Difference; {
                scad!(shape),
                holes
            });

            let extrude_params = LinExtrudeParams{
                height: thickness,
                .. Default::default()
            };
            scad!(LinearExtrude(extrude_params); shape)
        };
        let grooves = {
            let extrude_params = LinExtrudeParams{
                height: groove_depth,
                .. Default::default()
            };
            scad!(LinearExtrude(extrude_params); groove_outline)
        };

        scad!(Difference; {
            scad!(Union; {
                outer_extruded,
                front_nut_outer
            }),
            grooves,
            front_nut_cut
        })
    }

    fn side_plate_front_bracket(&self) -> ScadObject {
        let thickness = self.side_plate_thickness;
        let hole_top_offset = self.side_plate_front_screw_top_offset;
        let hole_diameter = 3.7;

        let width = 9.;
        let z_size = self.side_plate_arc_width + thickness * 2.;

        let shape = scad!(Union; {
            nut(width, z_size),
            centered_cube(
                vec3(width, hole_top_offset + thickness, z_size),
                (true, false, false)
            ),
        });

        let hole = scad!(Cylinder(z_size, Diameter(hole_diameter)));
        let cutout = scad!(Translate(vec3(0., 0., thickness)); {
            centered_cube(
                vec3(100., hole_top_offset*2., self.side_plate_arc_width),
                (true, true, false)
            )
        });

        scad!(Difference; {
            shape,
            hole,
            cutout
        })
    }
}


qstruct!(ServoMount() {
    servo_width: f32 = 12.25,
    servo_depth: f32 = 22.,
    servo_height: f32 = 28.,
    servo_tab_height: f32 = 4.6,
    boom_width: f32 = 11.,
    side_thickness: f32 = 2.,
    prop_clearance: f32 = 10.,
    top_offset: f32 = 7.,
});

impl ServoMount {
    fn full(&self) -> ScadObject {
        let extrude_params = LinExtrudeParams {
            height: self.servo_depth,
            .. Default::default()
        };

        let shape = scad!(LinearExtrude(extrude_params); {
                self.outline()
            });

        let to_keep = {
            let outline = {
                let points = vec![
                    vec2(-10., -10.),
                    vec2(-10., self.servo_height + self.boom_width),
                    vec2(
                        self.servo_depth - self.prop_clearance,
                        self.servo_height + self.boom_width
                    ),
                    vec2(
                        self.servo_depth - self.prop_clearance,
                        self.servo_height
                    ),
                    vec2(self.servo_depth, self.boom_width),
                    vec2(self.servo_depth, -10.),
                ];

                scad!(Polygon(PolygonParameters::new(points)))
            };

            let extrude_params = LinExtrudeParams {
                height: 100.,
                center: true,
                .. Default::default()
            };

            scad!(Rotate(-90., y_axis()); {
                scad!(LinearExtrude(extrude_params); outline)
            })
        };

        scad!(Intersection; {
            scad!(Union; {
                shape.clone(),
                scad!(Mirror(vec3(1., 0., 0.)); shape)
            }),
            to_keep
        })
    }
    fn outline(&self) -> ScadObject {
        let half_outer_outline = {
            let boom_x = self.boom_width / 2. + self.side_thickness;
            let servo_x = self.servo_width / 2. + self.side_thickness;
            let servo_y_start = self.boom_width;
            let top_tab_start = self.servo_height + self.boom_width
                    - self.side_thickness;
            let tab_length = 5.;

            vec!(
                vec2(0., -self.side_thickness),
                vec2(boom_x, -self.side_thickness),
                vec2(boom_x, self.boom_width / 2.),
                vec2(servo_x, servo_y_start),
                vec2(
                    servo_x + 1.,
                    servo_y_start + self.servo_tab_height
                ),
                vec2(servo_x, top_tab_start - self.top_offset),
                vec2(servo_x + tab_length, top_tab_start - self.top_offset),
                vec2(
                    servo_x + tab_length,
                    self.servo_height + self.boom_width - self.top_offset
                ),
                vec2(0., self.servo_height + self.boom_width - self.top_offset),
            )
        };

        let servo_mount_block_outline = {
            let top_width = 14.6;
            let bottom_width = 12.4;
            let height = 8.7;
            let bottom_offset = 2.;

            let points = vec!(
                    vec2(-bottom_width/2., -bottom_offset),
                    vec2(bottom_width/2., -bottom_offset),
                    vec2(top_width/2., height-bottom_offset),
                    vec2(-top_width/2., height-bottom_offset)
                );
            scad!(Translate2d(vec2(0., self.boom_width)); {
                scad!(Polygon(PolygonParameters::new(points)))
            })
        };

        let boom_outline = centered_square(
            vec2(self.boom_width, self.boom_width), (true, false)
        );

        let servo_outline = {
            let shape = centered_square(
                vec2(self.servo_width, self.servo_height), (true, false)
            );
            scad!(Translate2d(vec2(0., self.boom_width)); shape)
        };

        scad!(Difference; {
            scad!(Polygon(PolygonParameters::new(half_outer_outline))),
            boom_outline,
            servo_outline,
            servo_mount_block_outline
        })
    }

    fn flex_holder(&self) -> ScadObject {
        unimplemented!();
    }
}


fn get_camera_cushion() -> ScadObject
{
    let size = vec3(41., 16., 3.);

    let back_cushion_size = vec3(20., 3., size.x / 2.);

    let led_holes = {
        let cutout_size = vec3(10. * 2., 2., 100.);

        let cube = centered_cube(cutout_size, (true, false, false));
        let translated = scad!(Translate(vec3(size.x / 2., size.y - cutout_size.y, 0.)); cube);

        let mirrored = scad!(Mirror(vec3(1., 0., 0.));
        {
            translated.clone(),
        });
        scad!(Union;{
            translated,
            mirrored
        })
    };


    //scad!(Cube(size))
    scad!(Difference;
    {
        scad!(Union;{
            centered_cube(size, (true, false, false)),
            centered_cube(back_cushion_size, (true, false, false))
        }),
        led_holes
    })
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
    let z_pos = 25.;
    for i in 0..3
    {
        sfile.add_object(
                scad!(Translate(vec3(x_pos, 0., z_pos + (i as f32) * z_offset)); esc.clone())
            );
        sfile.add_object(
                scad!(Translate(vec3(x_pos, 0., z_pos + (i as f32) * z_offset + 3.)); holder.clone())
            );
    }
}


fn main() 
{
    let mut sfile = ScadFile::new();
    sfile.set_detail(20);

    //sfile.add_object(TricopterBody::new().get_body_bottom());
    // sfile.add_object(scad!(Translate(vec3(0., 0., 20.)); TricopterBody::new().get_body_top(true)));
    //sfile.add_object(get_vtx_mount());
    //sfile.add_object(EscStack::new().get_mid_section());
    //sfile.add_object(get_camera_cushion());
    // sfile.add_object(scad!(Translate(vec3(0., 0., 35.)); TricopterBody::new().get_canopy()));
    //sfile.add_object(NazeBoard::new().get_board());
    //sfile.add_object(scad!(Translate(vec3(0., 0., 27.));
    //                       add_named_color("brown", DysEsc::new().get_board())));
    //sfile.add_object(get_camera_water_seal(&BoardCamera::new(), &TricopterBody::new()));
    // sfile.add_object(TricopterBody::new().get_body_bottom());
    // sfile.add_object(TricopterBody::new().get_side_plate_mount());
    // sfile.add_object(TricopterBody::new().side_plate_shape());
    // sfile.add_object(TricopterBody::new().side_plate_front_bracket());
    sfile.add_object(ServoMount::new().full());
    /*
    sfile.add_object(
            add_named_color(
                "steelblue",
                scad!(Translate(vec3(0., 0., 30.)); NazeBoard::new().get_board())
            )
        );
    sfile.add_object(
            add_named_color(
                "dimgray",
                scad!(Translate(vec3(-40., 0., 30.)); 
                {
                    scad!(Rotate(-90., vec3(0., 1., 0.));
                    {
                        BoardCamera::new().get_model(),
                    })
                })
            )
        );

    test_esc_stack(&mut sfile);
    */


    sfile.write_to_file(String::from("out.scad"));

    add_text_to_history_file(&sfile.get_code(), "frame_history.scad");
}
