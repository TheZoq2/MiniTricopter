#[macro_use]
extern crate scad_generator;
extern crate scad_util;
extern crate nalgebra as na;

use scad_generator::*;

use std::string::String;

const SCREW_DIAMETER: f32 = 4.5;

fn naze_test() -> ScadObject
{
    let width = 35.;
    let height = 2.;

    let cube = scad!(Cube(vec3(width, width, height)));

    scad!(Translate(-vec3(width, width, 0.) / 2.); cube)
}


qstruct!(TricopterBody()
{
    radius: f32 = 80.,
    height: f32 = 4.,
    outer_width: f32 = 22.,
    inner_width: f32 = 50.,

    arm_width: f32 = 10.,

    front_block_x: f32 = 30.,
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
                    LinExtrudeParams{center:false, height:self.get_bottom_total_height(), ..Default::default()}
                );self.get_back_mount_block()),
            self.get_front_block(self.get_bottom_total_height()),
        });


        scad!(Difference;
        {
            body,
            self.get_front_arm_screw_holes(),
            self.get_back_screwholes(),
            self.get_front_screwholes(),
        })
    }

    pub fn get_body_top(&self) -> ScadObject
    {
        let body = scad!(Union;{
                scad!(LinearExtrude(
                    LinExtrudeParams{center:false, height:self.height, ..Default::default()}
                );self.get_body_shape()),
                self.get_front_block(self.height),
            });

        scad!(Difference;
        {
            body,
            self.get_front_arm_screw_holes(),
            self.get_back_screwholes(),
            self.get_front_screwholes(),
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
        let arm_padding = 0.5;
        let total_arm_width = self.arm_width + arm_padding;

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

    fn get_back_screwholes(&self) -> ScadObject
    {
        //The holes should be well in the mount block
        let x_offset = self.radius * 3. / 4.;

        scad!(Translate(vec3(x_offset, 0., 0.)); self.get_center_screwholes())
    }

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

    fn get_bottom_total_height(&self) -> f32
    {
        self.arm_width + self.height
    }

    fn get_front_block(&self, height: f32) -> ScadObject
    {
        let width = self.outer_width;
        let length = self.radius / 8.;
        let chamfer_radius = width / 10.;
        let x_offset = self.front_block_x;

        let mut cylinders = scad!(Hull);

        for point in vec!(
                (-length / 2., width),
                (-length / 2., -width),
                (length / 2., width), 
                (length / 2., -width)
            )
        {
            let cylinder = scad!(
                Cylinder(height, Radius(chamfer_radius))
            );

            let translated = scad!(Translate(vec3(point.0, point.1, 0.)); cylinder);

            cylinders.add_child(translated);
        }

        scad!(Translate(vec3(-x_offset, 0., 0.)); cylinders)
    }

    fn get_front_screwholes(&self) -> ScadObject
    {
        let x_offset = self.front_block_x;

        scad!(Translate(vec3(-x_offset, 0., 0.)); self.get_center_screwholes())
    }

}

fn main() 
{
    let mut sfile = ScadFile::new();
    sfile.set_detail(50);

    sfile.add_object(TricopterBody::new().get_body_bottom());
    sfile.add_object(scad!(Translate(vec3(0., 0., 20.)); TricopterBody::new().get_body_top()));
    sfile.add_object(scad!(Translate(vec3(0., 0., 30.)); naze_test()));

    sfile.write_to_file(String::from("out.scad"));
}
