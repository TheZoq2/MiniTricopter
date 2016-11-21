#[macro_use]
extern crate scad_generator;
extern crate scad_util;
extern crate nalgebra as na;

use scad_generator::*;

use std::string::String;

const SCREW_DIAMETER: f32 = 4.5;


qstruct!(TricopterBody()
{
    radius: f32 = 80.,
    height: f32 = 4.,
    outer_width: f32 = 22.,
    inner_width: f32 = 50.,

    arm_width: f32 = 10.,
});

impl TricopterBody
{
    pub fn get_body_bottom(&self) -> ScadObject
    {
        let body = scad!(Union;{
            scad!(LinearExtrude(
                    LinExtrudeParams{center:false, height:self.height, ..Default::default()}
                );self.get_body_shape()),
            scad!(LinearExtrude(
                    LinExtrudeParams{center:false, height:self.height + self.arm_width, ..Default::default()}
                );self.get_back_mount_block()),
        });


        scad!(Difference;
        {
            body,
            self.get_front_arm_screw_holes()
        })
    }

    fn get_body_shape(&self) -> ScadObject
    {
        let points = vec!(
            na::Vector2::new(0., -self.inner_width / 2.),
            na::Vector2::new(0., self.inner_width / 2.),
            na::Vector2::new(self.radius, self.outer_width / 2.),
            na::Vector2::new(self.radius, -self.outer_width / 2.),
        );
        
        let lines = vec!(vec!(0,1,2,3));

        let segment = scad!(Polygon(points, lines));

        let mut result = scad!(Union);
        for i in 0..3
        {
            result.add_child(scad!{Rotate(120. * i as f32, vec3(0.,0.,1.)); segment.clone()});
        }

        result
    }

    fn get_back_mount_block(&self) -> ScadObject
    {
        //Percentage of the radius that the block should take up
        let length_factor = 0.5;

        let start_width = self.outer_width + (self.inner_width - self.outer_width) * length_factor;

        //Adding some padding to the arm width
        let total_arm_width = self.arm_width + 0.5;

        let shape = {
            let points = vec!(
                    na::Vector2::new(self.radius * length_factor, total_arm_width/2.),
                    na::Vector2::new(self.radius * length_factor, start_width / 2.),
                    na::Vector2::new(self.radius, self.outer_width / 2.),
                    na::Vector2::new(self.radius, total_arm_width/2.),
                );
            let paths = vec!(vec!(0,1,2,3));

            scad!(Polygon(points, paths))
        };

        let mirrored = scad!(Mirror(vec3(0., 1., 0.));{shape.clone()});

        scad!(Union;{
            shape,
            mirrored
        })
    }

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
}

fn main() 
{
    let mut sfile = ScadFile::new();
    sfile.set_detail(50);

    sfile.add_object(TricopterBody::new().get_body_bottom());

    sfile.write_to_file(String::from("out.scad"));
}
