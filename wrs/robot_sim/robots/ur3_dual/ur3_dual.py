import os
import numpy as np
from panda3d.core import NodePath, CollisionNode, CollisionBox, Point3
import wrs.basis.robot_math as rm
import wrs.modeling.collision_model as mcm
import wrs.robot_sim._kinematics.jlchain as rkjlc
import wrs.robot_sim.robots.robot_interface as ri
import wrs.modeling.model_collection as mmc
import wrs.robot_sim.robots.ur3_dual.ur3_rtq85 as u3rtq85


class UR3Dual(ri.RobotInterface):

    def __init__(self, pos=np.zeros(3), rotmat=np.eye(3), name='ur3_dual', enable_cc=True):
        super().__init__(pos=pos, rotmat=rotmat, name=name, enable_cc=enable_cc)
        current_file_dir = os.path.dirname(__file__)
        # the body anchor
        self.body = rkjlc.rkjl.Anchor(name="ur3_dual_base", pos=self.pos, rotmat=self.rotmat, n_flange=2, n_lnk=3)
        self.body.loc_flange_pose_list[0] = [np.array([.0, .258485281374, 1.61051471863]),
                                             rm.rotmat_from_euler(-3.0 * np.pi / 4.0, 0, 0)]
        self.body.loc_flange_pose_list[1] = [np.array([.0, -.258485281374, 1.61051471863]),
                                             rm.rotmat_from_euler(3.0 * np.pi / 4.0, 0, 0) @
                                             rm.rotmat_from_euler(0, 0, np.pi)]
        self.body.lnk_list[0].name = "ur3_dual_base_link"
        self.body.lnk_list[0].cmodel = mcm.CollisionModel(
            initor=os.path.join(current_file_dir, "meshes", "ur3_dual_base.stl"),
            cdprim_type=mcm.const.CDPrimType.USER_DEFINED,
            userdef_cdprim_fn=self._base_cdprim)
        self.body.lnk_list[0].cmodel.rgba = rm.const.dim_gray
        self.body.lnk_list[1].name = "ur3_dual_frame_link"
        self.body.lnk_list[1].cmodel = mcm.CollisionModel(
            initor=os.path.join(current_file_dir, "meshes", "ur3_dual_frame.stl"),
            cdprim_type=mcm.const.CDPrimType.USER_DEFINED,
            userdef_cdprim_fn=self._base_cdprim)
        self.body.lnk_list[1].cmodel.rgba = rm.const.tab20_list[14]
        self.body.lnk_list[2].name = "ur3_dual_table_link"
        self.body.lnk_list[2].cmodel = mcm.gen_box(xyz_lengths=np.array([.8, 1.83, .024]), rgb=rm.const.tab20_list[3])
        self.body.lnk_list[2].loc_pos = np.array([.45, .0, 1.082])
        # left arm
        self.lft_arm = u3rtq85.UR3_Rtq85(pos=self.body.gl_flange_pose_list[0][0],
                                         rotmat=self.body.gl_flange_pose_list[0][1])
        self.lft_arm.home_conf = np.array(
            [np.pi / 12.0, -np.pi * 1.0 / 3.0, -np.pi * 2.0 / 3.0, -np.pi, -np.pi * 2.0 / 3.0, 0])
        # self.lft_arm.manipulator.jnts[0].motion_range = np.array([-np.pi * 5 / 3, -np.pi / 3])
        # self.lft_arm.manipulator.jnts[1].motion_range = np.array([-np.pi, 0])
        # self.lft_arm.manipulator.jnts[2].motion_range = np.array([0, np.pi])
        # self.lft_arm.manipulator.jnts[3].motion_range = np.array([np.pi / 6, np.pi * 7 / 6])
        # self.lft_arm.manipulator.jnts[4].motion_range = np.array([-np.pi, np.pi])
        # self.lft_arm.manipulator.jnts[5].motion_range = np.array([-np.pi, np.pi])
        self.lft_arm.manipulator.jlc.finalize(identifier_str=self.lft_arm.name + "_dual_lft")
        # right side
        self.rgt_arm = u3rtq85.UR3_Rtq85(pos=self.body.gl_flange_pose_list[1][0],
                                         rotmat=self.body.gl_flange_pose_list[1][1])
        self.rgt_arm.home_conf = np.array(
            [-np.pi / 12, -np.pi * 2.0 / 3.0, np.pi * 2.0 / 3.0, 0, np.pi * 2.0 / 3.0, np.pi])
        # self.rgt_arm.manipulator.jnts[0].motion_range = np.array([np.pi / 3, np.pi * 5 / 3])
        # self.rgt_arm.manipulator.jnts[1].motion_range = np.array([-np.pi, 0])
        # self.rgt_arm.manipulator.jnts[2].motion_range = np.array([-np.pi, 0])
        # self.rgt_arm.manipulator.jnts[3].motion_range = np.array([-np.pi * 5 / 6, np.pi / 6])
        # self.rgt_arm.manipulator.jnts[4].motion_range = np.array([-np.pi, np.pi])
        # self.rgt_arm.manipulator.jnts[5].motion_range = np.array([-np.pi, np.pi])
        self.rgt_arm.manipulator.jlc.finalize(identifier_str=self.rgt_arm.name + "_dual_rgt")
        if self.cc is not None:
            self.setup_cc()
        # go home
        self.goto_home_conf()

    @staticmethod
    def _base_cdprim(ex_radius=None):
        pdcnd = CollisionNode("ur3_dual_base")
        collision_primitive_c0 = CollisionBox(Point3(0.18, 0.0, 0.105),
                                              x=.61 + ex_radius, y=.41 + ex_radius, z=.105 + ex_radius)
        pdcnd.addSolid(collision_primitive_c0)
        collision_primitive_c1 = CollisionBox(Point3(0.0, 0.0, 0.4445),
                                              x=.321 + ex_radius, y=.321 + ex_radius, z=.2345 + ex_radius)
        pdcnd.addSolid(collision_primitive_c1)
        collision_primitive_c2 = CollisionBox(Point3(0.0, 0.0, 0.8895),
                                              x=.05 + ex_radius, y=.05 + ex_radius, z=.6795 + ex_radius)
        pdcnd.addSolid(collision_primitive_c2)
        collision_primitive_c3 = CollisionBox(Point3(0.0, 0.0, 1.619),
                                              x=.1 + ex_radius, y=.275 + ex_radius, z=.05 + ex_radius)
        pdcnd.addSolid(collision_primitive_c3)
        collision_primitive_l0 = CollisionBox(Point3(0.0, 0.300, 1.669),
                                              x=.1 + ex_radius, y=.029 + ex_radius, z=.021 + ex_radius)
        pdcnd.addSolid(collision_primitive_l0)
        collision_primitive_r0 = CollisionBox(Point3(0.0, -0.300, 1.669),
                                              x=.1 + ex_radius, y=.029 + ex_radius, z=.021 + ex_radius)
        pdcnd.addSolid(collision_primitive_r0)
        cdprim = NodePath("user_defined")
        cdprim.attachNewNode(pdcnd)
        return cdprim

    @property
    def n_dof(self):
        if self.delegator is None:
            return self.lft_arm.n_dof + self.rgt_arm.n_dof
        else:
            return self.delegator.n_dof

    def setup_cc(self):
        """
        author: weiwei
        date: 20240309
        """
        # dual arm
        # body
        bd = self.cc.add_cce(self.body.lnk_list[0], toggle_extcd=False)
        # left ee
        lft_elb = self.cc.add_cce(self.lft_arm.end_effector.palm.lnk_list[0])
        lft_ell0 = self.cc.add_cce(self.lft_arm.end_effector.lft_outer_jlc.jnts[0].lnk)
        lft_ell1 = self.cc.add_cce(self.lft_arm.end_effector.lft_outer_jlc.jnts[1].lnk)
        lft_ell2 = self.cc.add_cce(self.lft_arm.end_effector.lft_outer_jlc.jnts[2].lnk)
        lft_ell3 = self.cc.add_cce(self.lft_arm.end_effector.lft_outer_jlc.jnts[3].lnk)
        lft_elr0 = self.cc.add_cce(self.lft_arm.end_effector.rgt_outer_jlc.jnts[0].lnk)
        lft_elr1 = self.cc.add_cce(self.lft_arm.end_effector.rgt_outer_jlc.jnts[1].lnk)
        lft_elr2 = self.cc.add_cce(self.lft_arm.end_effector.rgt_outer_jlc.jnts[2].lnk)
        lft_elr3 = self.cc.add_cce(self.lft_arm.end_effector.rgt_outer_jlc.jnts[3].lnk)
        # left manipulator
        lft_ml0 = self.cc.add_cce(self.lft_arm.manipulator.jlc.jnts[0].lnk, toggle_extcd=False)
        lft_ml1 = self.cc.add_cce(self.lft_arm.manipulator.jlc.jnts[1].lnk)
        lft_ml2 = self.cc.add_cce(self.lft_arm.manipulator.jlc.jnts[2].lnk)
        lft_ml3 = self.cc.add_cce(self.lft_arm.manipulator.jlc.jnts[3].lnk)
        lft_ml4 = self.cc.add_cce(self.lft_arm.manipulator.jlc.jnts[4].lnk)
        lft_ml5 = self.cc.add_cce(self.lft_arm.manipulator.jlc.jnts[5].lnk)
        # right ee
        rgt_elb = self.cc.add_cce(self.rgt_arm.end_effector.palm.lnk_list[0])
        rgt_ell0 = self.cc.add_cce(self.rgt_arm.end_effector.lft_outer_jlc.jnts[0].lnk)
        rgt_ell1 = self.cc.add_cce(self.rgt_arm.end_effector.lft_outer_jlc.jnts[1].lnk)
        rgt_ell2 = self.cc.add_cce(self.rgt_arm.end_effector.lft_outer_jlc.jnts[2].lnk)
        rgt_ell3 = self.cc.add_cce(self.rgt_arm.end_effector.lft_outer_jlc.jnts[3].lnk)
        rgt_elr0 = self.cc.add_cce(self.rgt_arm.end_effector.rgt_outer_jlc.jnts[0].lnk)
        rgt_elr1 = self.cc.add_cce(self.rgt_arm.end_effector.rgt_outer_jlc.jnts[1].lnk)
        rgt_elr2 = self.cc.add_cce(self.rgt_arm.end_effector.rgt_outer_jlc.jnts[2].lnk)
        rgt_elr3 = self.cc.add_cce(self.rgt_arm.end_effector.rgt_outer_jlc.jnts[3].lnk)
        # right manipulator
        rgt_ml0 = self.cc.add_cce(self.rgt_arm.manipulator.jlc.jnts[0].lnk, toggle_extcd=False)
        rgt_ml1 = self.cc.add_cce(self.rgt_arm.manipulator.jlc.jnts[1].lnk)
        rgt_ml2 = self.cc.add_cce(self.rgt_arm.manipulator.jlc.jnts[2].lnk)
        rgt_ml3 = self.cc.add_cce(self.rgt_arm.manipulator.jlc.jnts[3].lnk)
        rgt_ml4 = self.cc.add_cce(self.rgt_arm.manipulator.jlc.jnts[4].lnk)
        rgt_ml5 = self.cc.add_cce(self.rgt_arm.manipulator.jlc.jnts[5].lnk)
        # first pairs
        from_list = [lft_elb, lft_ell0, lft_ell1, lft_ell2, lft_ell3, lft_elr0, lft_elr1, lft_elr2, lft_elr3,
                     rgt_elb, rgt_ell0, rgt_ell1, rgt_ell2, rgt_ell3, rgt_elr0, rgt_elr1, rgt_elr2, rgt_elr3]
        into_list = [bd, lft_ml0, lft_ml1, lft_ml2, rgt_ml0, rgt_ml1, rgt_ml2]
        self.cc.set_cdpair_by_ids(from_list, into_list)
        # second pairs
        from_list = [lft_ml2, rgt_ml2]
        into_list = [bd, lft_ml0, rgt_ml0]
        self.cc.set_cdpair_by_ids(from_list, into_list)
        # third pairs
        from_list = [lft_ml2, lft_ml3, lft_ml4, lft_ml5, lft_elb, lft_ell0, lft_ell1, lft_ell2, lft_ell3, lft_elr0,
                     lft_elr1, lft_elr2, lft_elr3]
        into_list = [rgt_ml2, rgt_ml3, rgt_ml4, rgt_ml5, rgt_elb, rgt_ell0, rgt_ell1, rgt_ell2, rgt_ell3, rgt_elr0,
                     rgt_elr1, rgt_elr2, rgt_elr3]
        self.cc.set_cdpair_by_ids(from_list, into_list)
        # point low-level cc to the high-level one
        self.lft_arm.cc = self.cc
        self.rgt_arm.cc = self.cc

    def use_both(self):
        self.delegator = None

    def use_lft(self):
        self.delegator = self.lft_arm

    def use_rgt(self):
        self.delegator = self.rgt_arm

    def backup_state(self):
        if self.delegator is None:
            self.rgt_arm.backup_state()
            self.lft_arm.backup_state()
        else:
            self.delegator.backup_state()

    def restore_state(self):
        if self.delegator is None:
            self.rgt_arm.restore_state()
            self.lft_arm.restore_state()
        else:
            self.delegator.restore_state()

    def fix_to(self, pos, rotmat):
        self.pos = pos
        self.rotmat = rotmat
        self.body.pos = self.pos
        self.body.rotmat = self.rotmat
        self.lft_arm.fix_to(pos=self.body.gl_flange_pose_list[0][0],
                            rotmat=self.body.gl_flange_pose_list[0][1])
        self.rgt_arm.fix_to(pos=self.body.gl_flange_pose_list[1][0],
                            rotmat=self.body.gl_flange_pose_list[1][1])

    def fk(self, jnt_values, toggle_jacobian=False):
        if self.delegator is None:
            raise AttributeError("FK is not available in multi-arm mode.")
        else:
            return self.delegator.fk(jnt_values=jnt_values, toggle_jacobian=toggle_jacobian)

    def ik(self, tgt_pos, tgt_rotmat, seed_jnt_values=None, obstacle_list=None, toggle_dbg=False):
        if self.delegator is None:
            raise AttributeError("IK is not available in multi-arm mode.")
        else:
            candidates = self.delegator.ik(tgt_pos=tgt_pos, tgt_rotmat=tgt_rotmat, seed_jnt_values=seed_jnt_values,
                                           option="multiple", toggle_dbg=toggle_dbg)
            result = None
            self.delegator.backup_state()
            for jnt_values in candidates:
                self.delegator.goto_given_conf(jnt_values=jnt_values)
                if self.is_collided(obstacle_list=obstacle_list, toggle_contacts=False):
                    continue
                else:
                    result = jnt_values
                    break
            self.delegator.restore_state()
            return result

    def goto_given_conf(self, jnt_values, ee_values=None):
        """
        :param jnt_values: nparray 1x14, 0:7lft, 7:14rgt
        :return:
        author: weiwei
        date: 20240307
        """
        if self.delegator is None:
            if len(jnt_values) != self.lft_arm.manipulator.n_dof + self.rgt_arm.manipulator.n_dof:
                raise ValueError("The given joint values do not match total n_dof")
            self.lft_arm.goto_given_conf(jnt_values=jnt_values[:self.lft_arm.manipulator.n_dof])
            self.rgt_arm.goto_given_conf(jnt_values=jnt_values[self.rgt_arm.manipulator.n_dof:])  # TODO
        else:
            self.delegator.goto_given_conf(jnt_values=jnt_values, ee_values=ee_values)

    def goto_home_conf(self):
        if self.delegator is None:
            self.lft_arm.goto_home_conf()
            self.rgt_arm.goto_home_conf()
        else:
            self.delegator.goto_home_conf()

    def get_jnt_values(self):
        if self.delegator is None:
            return np.concatenate((self.lft_arm.get_jnt_values(), self.rgt_arm.get_jnt_values()))
        else:
            return self.delegator.get_jnt_values()

    def rand_conf(self):
        """
        :return:
        author: weiwei
        date: 20210406
        """
        if self.delegator is None:
            return np.concatenate((self.lft_arm.rand_conf(), self.rgt_arm.rand_conf()))
        else:
            return self.delegator.rand_conf()

    def are_jnts_in_ranges(self, jnt_values):
        if self.delegator is None:
            return self.lft_arm.are_jnts_in_ranges(
                jnt_values=jnt_values[:self.lft_arm.manipulator.n_dof]) and self.rgt_arm.are_jnts_in_ranges(
                jnt_values=jnt_values[self.rgt_arm.manipulator.n_dof:])
        else:
            return self.delegator.are_jnts_in_ranges(jnt_values=jnt_values)

    def get_jaw_width(self):
        return self.get_ee_values()

    def change_jaw_width(self, jaw_width):
        self.change_ee_values(ee_values=jaw_width)

    def is_collided(self, obstacle_list=None, other_robot_list=None, toggle_contacts=False):
        """
        Interface for "is cdprimit collided", must be implemented in child class
        :param obstacle_list:
        :param other_robot_list:
        :param toggle_contacts: debug
        :return: see CollisionChecker is_collided for details
        author: weiwei
        date: 20240307
        """
        collision_info = self.cc.is_collided(obstacle_list=obstacle_list,
                                             other_robot_list=other_robot_list,
                                             toggle_contacts=toggle_contacts)
        return collision_info

    def gen_stickmodel(self,
                       toggle_tcp_frame=False,
                       toggle_jnt_frames=False,
                       toggle_flange_frame=False,
                       name='yumi_stickmodel'):
        m_col = mmc.ModelCollection(name=name)
        self.body.gen_stickmodel(toggle_root_frame=toggle_jnt_frames,
                                 toggle_flange_frame=toggle_flange_frame).attach_to(m_col)
        self.lft_arm.gen_stickmodel(toggle_tcp_frame=toggle_tcp_frame,
                                    toggle_jnt_frames=toggle_jnt_frames,
                                    toggle_flange_frame=toggle_flange_frame,
                                    name=name + "_lft_arm").attach_to(m_col)
        self.rgt_arm.gen_stickmodel(toggle_tcp_frame=toggle_tcp_frame,
                                    toggle_jnt_frames=toggle_jnt_frames,
                                    toggle_flange_frame=toggle_flange_frame,
                                    name=name + "_rgt_arm").attach_to(m_col)
        return m_col

    def gen_meshmodel(self,
                      rgb=None,
                      alpha=None,
                      toggle_tcp_frame=False,
                      toggle_jnt_frames=False,
                      toggle_flange_frame=False,
                      toggle_cdprim=False,
                      toggle_cdmesh=False,
                      name='yumi_meshmodel'):
        m_col = mmc.ModelCollection(name=name)
        self.body.gen_meshmodel(rgb=rgb, alpha=alpha, toggle_flange_frame=toggle_flange_frame,
                                toggle_root_frame=toggle_jnt_frames, toggle_cdprim=toggle_cdprim,
                                toggle_cdmesh=toggle_cdmesh, name=name + "_body").attach_to(m_col)
        self.lft_arm.gen_meshmodel(rgb=rgb,
                                   alpha=alpha,
                                   toggle_tcp_frame=toggle_tcp_frame,
                                   toggle_jnt_frames=toggle_jnt_frames,
                                   toggle_flange_frame=toggle_flange_frame,
                                   toggle_cdprim=toggle_cdprim,
                                   toggle_cdmesh=toggle_cdmesh,
                                   name=name + "_lft_arm").attach_to(m_col)
        self.rgt_arm.gen_meshmodel(rgb=rgb,
                                   alpha=alpha,
                                   toggle_tcp_frame=toggle_tcp_frame,
                                   toggle_jnt_frames=toggle_jnt_frames,
                                   toggle_flange_frame=toggle_flange_frame,
                                   toggle_cdprim=toggle_cdprim,
                                   toggle_cdmesh=toggle_cdmesh,
                                   name=name + "_rgt_arm").attach_to(m_col)
        return m_col


if __name__ == '__main__':
    import wrs.visualization.panda.world as wd

    base = wd.World(cam_pos=[3, 1, 3], lookat_pos=[0, 0, 1])
    mcm.mgm.gen_frame().attach_to(base)
    u3d = UR3Dual()
    u3d_meshmodel = u3d.gen_meshmodel(toggle_tcp_frame=True, toggle_cdprim=True)
    u3d_meshmodel.attach_to(base)
    base.run()
