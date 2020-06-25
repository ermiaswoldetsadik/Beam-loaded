using System;
using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;
using Frame3D;
using MathLib;

namespace WindowsApplication1
{
    public partial class Form1 : Form
    {
        private const string LicenseInfo = "Demo";

        #region Example 1
        private void Example_1(object sender, EventArgs e)
        {
            //New model definition            
            Model Model = new Model();
            Model.LicenseInfo = LicenseInfo;

            //-------MATERIAL DEFINITION-------

            //Create a new material for concrete
            Material matConcrete = new Material();
            matConcrete.Name = "Concrete";//Material name
            matConcrete.Density = 2.5;//density in mass units/m3, for example tn/m3
            matConcrete.G = 11538461;//shear modulus
            matConcrete.E = 30000000;//elasticity modulus

            //-------SECTIONS DEFINITION-------

            //Create a new beam section of dimensions 40cmx80xm
            FrameElementSection secBeam40_80 = new FrameElementSection();
            secBeam40_80.Name = "Beam40/80";//section name
            secBeam40_80.A = 0.4 * 0.8;//section area
            secBeam40_80.Iy = 0.4 * 0.8 * 0.8 * 0.8 / 12;//inertia moment about local y axis
            secBeam40_80.Iz = 0.8 * 0.4 * 0.4 * 0.4 / 12;//inertia moment about local z axis
            secBeam40_80.It = 0.0117248;//torsional constant
            secBeam40_80.b = 0.40;//section width
            secBeam40_80.h = 0.80;//section height

            //-------MODEL GEOMETRY AND LOADS DEFINITION-------

            //Create node n1
            Frame3D.SuperNode n1 = new Frame3D.SuperNode(1, 0, 0, 0);
            n1.dof1constraint = true;//delete
            n1.dof2constraint = true;//translational constraint in direction y at local system of node
            n1.dof3constraint = true;//translational constraint in direction z at local system of node
            n1.dof4constraint = true;//rotational constraint in direction x at local system of node
            n1.dof5constraint = true;//rotational constraint in direction y at local system of node
            Model.InputNodes.Add(n1);

            //Create node n2            
            Frame3D.SuperNode n2 = new Frame3D.SuperNode(2, 5, 0, 0);
            n2.dof1constraint = true;//translational constraint in direction x at local system of node
            n2.dof2constraint = true;//translational constraint in direction y at local system of node
            n2.dof3constraint = true;//translational constraint in direction z at local system of node
            n2.dof4constraint = true;//rotational constraint in direction x at local system of node
            n2.dof5constraint = true;//rotational constraint in direction y at local system of node
            Model.InputNodes.Add(n2);

            //Create frame element 1
            //Note that the 4th argument specifies the auxiliary point that lies in the xy plane that is formed by the x and y axes in the local element system
            FrameSuperElement el1 = new FrameSuperElement(1, n1, n2, new Geometry.XYZ(0, 0, 1), matConcrete, secBeam40_80, new MemberReleases(), new MemberReleases(), false, false);

            LinearLoadCaseForSuperFrameElement lc1 = new LinearLoadCaseForSuperFrameElement("lc1", LoadCaseType.DEAD);
            lc1.UniformLoad.UniformLoadsY.Add(new FrameSuperUniformLoad(0, 1, -10, -10, LoadDefinitionFromStartingNode.Relatively, LoadCordinateSystem.Global));
            lc1.PointLoad.PointLoadsY.Add(new SuperPointLoad(3.5, -50, LoadDefinitionFromStartingNode.Absolutely, LoadCordinateSystem.Global));
            el1.LinearLoadCasesList.Add(lc1);

            LinearLoadCaseForSuperFrameElement lc2 = new LinearLoadCaseForSuperFrameElement("lc2", LoadCaseType.LIVE);
            lc2.UniformLoad.UniformLoadsY.Add(new FrameSuperUniformLoad(0, 1, -5, -5, LoadDefinitionFromStartingNode.Relatively, LoadCordinateSystem.Global));
            el1.LinearLoadCasesList.Add(lc2);

            LinearLoadCaseForSuperFrameElement lc3 = new LinearLoadCaseForSuperFrameElement("lc3", LoadCaseType.LIVE);
            lc3.UniformLoad.UniformLoadsY.Add(new FrameSuperUniformLoad(0, 1, -1, -1, LoadDefinitionFromStartingNode.Relatively, LoadCordinateSystem.Global));
            el1.LinearLoadCasesList.Add(lc3);

            Model.InputFiniteElements.Add(el1);

            //-------SOLUTION PHASE-------
            Model.Solve();

            //-------OBTAIN RESULTS-------
            double[] Min, Max;//The combination results will be saved in these arrays
            //Note that the definition of two arrays for minimum and maximum combination results is required
            //For combination type "ADD", Min and Max values are always equal

            //Reactions (All are defined in the node local system)
            //Rections for load case lc1
            n1.GetReactionsForLoadCase("lc1", out Min, out Max, 0);
            double n1_Rty_lc1 = Max[1];
            n2.GetReactionsForLoadCase("lc1", out Min, out Max, 0);
            double n2_Rty_lc1 = Max[1];

            //Rections for load case lc2
            n1.GetReactionsForLoadCase("lc2", out Min, out Max, 0);
            double n1_Rty_lc2 = Max[1];
            n2.GetReactionsForLoadCase("lc2", out Min, out Max, 0);
            double n2_Rty_lc2 = Max[1];

            //Rections for load case lc13
            n1.GetReactionsForLoadCase("lc3", out Min, out Max, 0);
            double n1_Rty_lc3 = Max[1];
            n2.GetReactionsForLoadCase("lc3", out Min, out Max, 0);
            double n2_Rty_lc3 = Max[1];

            //Node Displacements (All are defined in the node local system)
            //Note that constained degrees of freedom have zero displacements
            n1.GetNodalDisplacementsForLoadCase("lc1", out Min, out Max, 0);
            double[] n1_Disp = Max;
            n2.GetNodalDisplacementsForLoadCase("lc1", out Min, out Max, 0);
            double[] n2_Disp = Max;

            //Element internal forces and displacements
            el1.GetInternalForcesForLoadCase(0, "lc1", out Min, out Max, 0); //Internal forces at the start of the member
            double[] forces_along_member_left = Max;
            el1.GetInternalForcesForLoadCase(2.5, "lc1", out Min, out Max, 0);//Internal forces at the middle of the member
            double[] forces_along_member_middle = Max;
            el1.GetInternalForcesForLoadCase(5, "lc1", out Min, out Max, 0);//Internal forces at the end of the member
            double[] forces_along_member_right = Max;

            el1.GetDisplacementsForLoadCase(0, "lc1", out Min, out Max, 0); //Internal displacements at the start of the member
            double[] disps_along_member_left = Max;
            el1.GetDisplacementsForLoadCase(2.5, "lc1", out Min, out Max, 0);//Internal displacements at the middle of the member
            double[] disps_along_member_middle = Max;
            el1.GetDisplacementsForLoadCase(5, "lc1", out Min, out Max, 0);//Internal displacements at the end of the member
            double[] disps_along_member_right = Max;

            //Creation of a load combination
            //Note that load combinations can also be defined after analysis has been completed
            //A load combination for 2.00 lc1 - 0.5 lc2 is created, as follows:
            LoadCombination LCombo = new LoadCombination("combination", ComboType.ADD);
            LCombo.Items.Add(new LoadCaseWithFactor("lc1", 2));
            LCombo.Items.Add(new LoadCaseWithFactor("lc2", -0.5));

            //All result data can be now obtained for the combination in the same way as for the load cases
            //for example, get first node reactions:
            n1.GetReactionsForLoadCombination(LCombo, out Min, out Max);
        }
        #endregion

        #region Example 2
        private void Example_2(object sender, EventArgs e)
        {
            //New model definition            
            Model Model = new Model();
            Model.LicenseInfo = LicenseInfo;

            //-------MATERIAL DEFINITION-------

            //Create a new material for concrete
            Material matConcrete = new Material();
            matConcrete.Name = "Concrete";//Material name
            matConcrete.Density = 2.5;//density in mass units/m3, for example tn/m3
            matConcrete.G = 11538461;//shear modulus
            matConcrete.E = 30000000;//elasticity modulus

            //-------SECTIONS DEFINITION-------

            //Create a new beam section of dimensions 40cmx80xm
            FrameElementSection secBeam40_80 = new FrameElementSection();
            secBeam40_80.Name = "Beam40/80";//section name
            secBeam40_80.A = 0.4 * 0.8;//section area
            secBeam40_80.Iy = 0.4 * 0.8 * 0.8 * 0.8 / 12;//inertia moment about local y axis
            secBeam40_80.Iz = 0.8 * 0.4 * 0.4 * 0.4 / 12;//inertia moment about local z axis
            secBeam40_80.It = 0.0117248;//torsional constant
            secBeam40_80.b = 0.40;//section height
            secBeam40_80.h = 0.80;//section height

            //-------MODEL GEOMETRY AND LOADS DEFINITION-------

            //Create node n1
            Frame3D.SuperNode n1 = new Frame3D.SuperNode(1, 0, 0, 0);
            n1.dof1constraint = true;//translational constraint in direction y at local system of node
            n1.dof2constraint = true;//translational constraint in direction y at local system of node
            n1.dof3constraint = true;//translational constraint in direction z at local system of node
            n1.dof4constraint = true;//rotational constraint in direction x at local system of node
            n1.dof5constraint = true;//rotational constraint in direction y at local system of node
            n1.dof6constraint = true;//rotational constraint in direction y at local system of node
            Model.InputNodes.Add(n1);

            //Create node n2            
            Frame3D.SuperNode n2 = new Frame3D.SuperNode(2, 5, 0, 0);
            n2.dof1constraint = true;//translational constraint in direction x at local system of node
            n2.dof2constraint = true;//translational constraint in direction y at local system of node
            n2.dof3constraint = true;//translational constraint in direction z at local system of node
            n2.dof4constraint = true;//rotational constraint in direction x at local system of node
            n2.dof5constraint = true;//rotational constraint in direction y at local system of node
            n2.dof6constraint = true;//rotational constraint in direction y at local system of node
            Model.InputNodes.Add(n2);

            //Create frame element 1
            //Note the definition of the auxiliary point: Geometry.XYZ(0, Math.Tan(30/180*Math.PI), 1)
            //It shows that the frame will be inserted properly (rotated about its longitudinal axis)
            FrameSuperElement el1 = new FrameSuperElement(1, n1, n2, new Geometry.XYZ(0, 0, 1), matConcrete, secBeam40_80, new MemberReleases(), new MemberReleases(), false, false);

            LinearLoadCaseForSuperFrameElement lc1 = new LinearLoadCaseForSuperFrameElement("lc1", LoadCaseType.DEAD);
            lc1.UniformLoad.UniformLoadsY.Add(new FrameSuperUniformLoad(0, 1, -10, -10, LoadDefinitionFromStartingNode.Relatively, LoadCordinateSystem.Global));
            el1.LinearLoadCasesList.Add(lc1);

            Model.InputFiniteElements.Add(el1);


            //-------SOLUTION PHASE-------
            Model.Solve();


            //-------OBTAIN RESULTS-------
            double[] Min, Max;

            //Reactions
            //Rections for load case lc1
            n1.GetReactionsForLoadCase("lc1", out Min, out Max, 0);
            double[] n1_R_lc1 = Max;
            n2.GetReactionsForLoadCase("lc1", out Min, out Max, 0);
            double[] n2_R_lc1 = Max;

            //Note that element forces are now different, shear force acts on both y and z directions in element local system
            el1.GetInternalForcesForLoadCase(0, "lc1", out Min, out Max, 0);//Internal forces at the start of the member
            double[] forces_along_member_left = Max;
            el1.GetInternalForcesForLoadCase(2.5, "lc1", out Min, out Max, 0);//Internal forces at the middle of the member
            double[] forces_along_member_middle = Max;
            el1.GetInternalForcesForLoadCase(5, "lc1", out Min, out Max, 0);//Internal forces at the end of the member
            double[] forces_along_member_right = Max;

            el1.GetDisplacementsForLoadCase(0, "lc1", out Min, out Max, 0);//Internal displacements at the start of the member
            double[] disps_along_member_left = Max;
            el1.GetDisplacementsForLoadCase(2.5, "lc1", out Min, out Max, 0);//Internal displacements at the middle of the member
            double[] disps_along_member_middle = Max;
            el1.GetDisplacementsForLoadCase(5, "lc1", out Min, out Max, 0);//Internal displacements at the end of the member
            double[] disps_along_member_right = Max;
        }
        #endregion

        #region Example 3
        private void Example_3(object sender, EventArgs e)
        {
            //New model definition            
            Model Model = new Model();
            Model.LicenseInfo = LicenseInfo;

            //-------MATERIAL DEFINITION-------

            //Create a new material for concrete
            Material matConcrete = new Material();
            matConcrete.Name = "Concrete";//Material name
            matConcrete.Density = 2.5;//density in mass units/m3, for example tn/m3
            matConcrete.G = 11538461;//shear modulus
            matConcrete.E = 30000000;//elasticity modulus

            //-------SECTIONS DEFINITION-------

            //Create a new beam section of dimensions 40cmx80xm
            FrameElementSection secBeam40_80 = new FrameElementSection();
            secBeam40_80.Name = "Beam40/80";//section name
            secBeam40_80.A = 0.4 * 0.8;//section area
            secBeam40_80.Iy = 0.4 * 0.8 * 0.8 * 0.8 / 12;//inertia moment about local y axis
            secBeam40_80.Iz = 0.8 * 0.4 * 0.4 * 0.4 / 12;//inertia moment about local z axis
            secBeam40_80.It = 0.0117248;//torsional constant
            secBeam40_80.b = 0.40;//section height
            secBeam40_80.h = 0.80;//section height

            //-------MODEL GEOMETRY AND LOADS DEFINITION-------

            //Create node n1, the local coordinate system of the node is assigned, which means that it is different from the default global system.
            //In order to define the new system, a new LocalCoordinateSystem is passed in the corresponding constructor of SuperNode object
            //The first two point of this constructor define the local x axis of the node system and the third one defines the coordinates of an auxiliary
            //point that lies in local XY plane
            Frame3D.SuperNode n1 = new Frame3D.SuperNode(1, 0, 0, 0, new LocalCoordinateSystem(new Geometry.XYZ(0, 0, 0), new Geometry.XYZ(1, Math.Tan(-Math.PI / 6), 0), new Geometry.XYZ(1, Math.Tan(60.0 / 180 * Math.PI), 0)));
            n1.dof2constraint = true;//translational constraint in direction y at local system (which was defined previously) of node
            n1.dof3constraint = true;//translational constraint in direction z at local system of node
            n1.dof4constraint = true;//rotational constraint in direction x at local system of node
            n1.dof5constraint = true;//rotational constraint in direction y at local system of node
            Model.InputNodes.Add(n1);

            //Create node n2            
            Frame3D.SuperNode n2 = new Frame3D.SuperNode(2, 5, 0, 0);
            n2.dof1constraint = true;//translational constraint in direction x at local system of node
            n2.dof2constraint = true;//translational constraint in direction y at local system of node
            n2.dof3constraint = true;//translational constraint in direction z at local system of node
            n2.dof4constraint = true;//rotational constraint in direction x at local system of node
            n2.dof5constraint = true;//rotational constraint in direction y at local system of node
            Model.InputNodes.Add(n2);

            //Create frame element 1           
            FrameSuperElement el1 = new FrameSuperElement(1, n1, n2, new Geometry.XYZ(0, 0, 1), matConcrete, secBeam40_80, new MemberReleases(), new MemberReleases(), false, false);

            LinearLoadCaseForSuperFrameElement lc1 = new LinearLoadCaseForSuperFrameElement("lc1", LoadCaseType.DEAD);
            lc1.UniformLoad.UniformLoadsY.Add(new FrameSuperUniformLoad(0, 1, -10, -10, LoadDefinitionFromStartingNode.Relatively, LoadCordinateSystem.Global));
            el1.LinearLoadCasesList.Add(lc1);

            Model.InputFiniteElements.Add(el1);

            //-------SOLUTION PHASE-------
            Model.Solve();

            //-------OBTAIN RESULTS-------
            double[] Min, Max;

            //Support reactions (Note that they are defined in the node local system)
            n1.GetReactionsForLoadCase("lc1", out Min, out Max, 0);
            double n1_Rty_lc1 = Max[1];
            n2.GetReactionsForLoadCase("lc1", out Min, out Max, 0);//Axial force is acting on the element because of the skew support at node 1
            double n2_Rtx_lc1 = Max[0];
            n2.GetReactionsForLoadCase("lc1", out Min, out Max, 0);
            double n2_Rty_lc1 = Max[1];
        }
        #endregion

        #region Example 4
        private void Example_4(object sender, EventArgs e)
        {
            //New model definition            
            Model Model = new Model();
            Model.LicenseInfo = LicenseInfo;

            //-------MATERIAL DEFINITION-------

            //Create a new material for concrete
            Material matConcrete = new Material();
            matConcrete.Name = "Concrete";//Material name
            matConcrete.Density = 2.5;//density in mass units/m3, for example tn/m3
            matConcrete.G = 11538461;//shear modulus
            matConcrete.E = 30000000;//elasticity modulus

            //-------SECTIONS DEFINITION-------

            //Create a new beam section of dimensions 40cmx80xm
            FrameElementSection secBeam40_80 = new FrameElementSection();
            secBeam40_80.Name = "Beam40/80";//section name
            secBeam40_80.A = 0.4 * 0.8;//section area
            secBeam40_80.Iy = 0.4 * 0.8 * 0.8 * 0.8 / 12;//inertia moment about local y axis
            secBeam40_80.Iz = 0.8 * 0.4 * 0.4 * 0.4 / 12;//inertia moment about local z axis
            secBeam40_80.It = 0.0117248;//torsional constant
            secBeam40_80.b = 0.40;//section height
            secBeam40_80.h = 0.80;//section height

            //-------MODEL GEOMETRY AND LOADS DEFINITION-------

            //Create node n1            
            Frame3D.SuperNode n1 = new Frame3D.SuperNode(1, 0, 0, 0);
            n1.dof3constraint = true;//translational constraint in direction z at local system of node
            n1.dof4constraint = true;//rotational constraint in direction x at local system of node
            n1.dof5constraint = true;//rotational constraint in direction y at local system of node
            n1.Kdof2 = 5000;//Translational spring constant of partial support at y direction of local node system (units: force/length, for example kN/m)
            n1.Kdof6 = 30000;//Rotational spring constant of partial support about z direction of local node system (units: moment/rotation, for example kNm/rad)
            Model.InputNodes.Add(n1);

            //Create node n2            
            Frame3D.SuperNode n2 = new Frame3D.SuperNode(2, 5, 0, 0);
            n2.dof1constraint = true;//translational constraint in direction x at local system of node
            n2.dof2constraint = true;//translational constraint in direction y at local system of node
            n2.dof3constraint = true;//translational constraint in direction z at local system of node
            n2.dof4constraint = true;//rotational constraint in direction x at local system of node
            n2.dof5constraint = true;//rotational constraint in direction y at local system of node
            n2.dof6constraint = true;//rotational constraint in direction z at local system of node
            Model.InputNodes.Add(n2);

            //Create frame element 1 
            FrameSuperElement el1 = new FrameSuperElement(1, n1, n2, new Geometry.XYZ(0, 0, 1), matConcrete, secBeam40_80, new MemberReleases(), new MemberReleases(), false, false);

            LinearLoadCaseForSuperFrameElement lc1 = new LinearLoadCaseForSuperFrameElement("lc1", LoadCaseType.DEAD);
            lc1.UniformLoad.UniformLoadsY.Add(new FrameSuperUniformLoad(0, 1, -10, -10, LoadDefinitionFromStartingNode.Relatively, LoadCordinateSystem.Global));
            el1.LinearLoadCasesList.Add(lc1);

            Model.InputFiniteElements.Add(el1);

            //-------SOLUTION PHASE-------
            Model.Solve();

            //-------OBTAIN RESULTS-------
            double[] Min, Max;

            //Spring reactions can be obtained from the corresponding Method, as follows
            //Spring reactions, as node reactions, as reported in the node local system
            n1.GetSpringReactionsForLoadCase("lc1", out Min, out Max, 0);
            double n1_Rty_lc1 = Max[1];
            n1.GetSpringReactionsForLoadCase("lc1", out Min, out Max, 0);
            double n1_Rrz_lc1 = Max[5];
            n2.GetReactionsForLoadCase("lc1", out Min, out Max, 0);
            double n2_Rty_lc1 = Max[1];
            n2.GetReactionsForLoadCase("lc1", out Min, out Max, 0);
            double n2_Rzz_lc1 = Max[5];
        }
        #endregion

        #region Example 5
        private void Example_5(object sender, EventArgs e)
        {
            //New model definition            
            Model Model = new Model();
            Model.LicenseInfo = LicenseInfo;

            //-------MATERIAL DEFINITION-------

            //Create a new material for concrete
            Material matConcrete = new Material();
            matConcrete.Name = "Concrete";//Material name
            matConcrete.Density = 2.5;//density in mass units/m3, for example tn/m3
            matConcrete.G = 11538461;//shear modulus
            matConcrete.E = 30000000;//elasticity modulus

            //-------SECTIONS DEFINITION-------

            //Create a new beam section of dimensions 30cmx70xm
            FrameElementSection secBeam30_70 = new FrameElementSection();
            secBeam30_70.Name = "Beam30/70";//section name
            secBeam30_70.A = 0.3 * 0.7;//section area
            secBeam30_70.Iy = 0.3 * 0.7 * 0.7 * 0.7 / 12;//inertia moment about local y axis
            secBeam30_70.Iz = 0.8 * 0.3 * 0.3 * 0.3 / 12;//inertia moment about local z axis
            secBeam30_70.It = 4.347e-3;//torsional constant
            secBeam30_70.b = 0.30;//section height
            secBeam30_70.h = 0.70;//section height

            //Create a new beam section of dimensions 50cmx50xm           
            FrameElementSection secColumn50_50 = new FrameElementSection();
            secColumn50_50.Name = "Column50/50";  //section name        
            secColumn50_50.A = 0.5 * 0.5;//section area
            secColumn50_50.Iy = 0.5 * 0.5 * 0.5 * 0.5 / 12;//inertia moment about local y axis
            secColumn50_50.Iz = 0.5 * 0.5 * 0.5 * 0.5 / 12;//inertia moment about local z axis
            secColumn50_50.It = 8.8125e-3;
            secColumn50_50.b = 0.50;//section height
            secColumn50_50.h = 0.50;//section height

            //-------MODEL GEOMETRY AND LOADS DEFINITION-------

            //Create node n1 
            Frame3D.SuperNode n1 = new Frame3D.SuperNode(1, 0, 0, 0);
            n1.dof1constraint = true;//translational constraint in direction x at local system of node
            n1.dof2constraint = true;//translational constraint in direction y at local system of node
            n1.dof3constraint = true;//translational constraint in direction z at local system of node
            n1.dof4constraint = true;//rotational constraint in direction x at local system of node
            n1.dof5constraint = true;//rotational constraint in direction y at local system of node
            n1.dof6constraint = true;//rotational constraint in direction z at local system of node
            Model.InputNodes.Add(n1);

            //Create node n2
            Frame3D.SuperNode n2 = new Frame3D.SuperNode(2, 0, 4, 0);
            Model.InputNodes.Add(n2);

            //Create node n3
            Frame3D.SuperNode n3 = new Frame3D.SuperNode(3, 5, 4, 0);
            Model.InputNodes.Add(n3);

            //Create node n4
            Frame3D.SuperNode n4 = new Frame3D.SuperNode(4, 5, 0, 0);
            n4.dof1constraint = true;//translational constraint in direction x at local system of node
            n4.dof2constraint = true;//translational constraint in direction y at local system of node
            n4.dof3constraint = true;//translational constraint in direction z at local system of node
            n4.dof4constraint = true;//rotational constraint in direction x at local system of node
            n4.dof5constraint = true;//rotational constraint in direction y at local system of node
            n4.dof6constraint = true;//rotational constraint in direction z at local system of node
            Model.InputNodes.Add(n4);

            //Create frame element 1 
            FrameSuperElement el1 = new FrameSuperElement(1, n1, n2, new Geometry.XYZ(0, 0, 1), matConcrete, secColumn50_50, new MemberReleases(), new MemberReleases(), false, false);
            Model.InputFiniteElements.Add(el1);

            //Create a MemberRelases object. Release are defined in element local coordinate system.
            MemberReleases PartialRelease = new MemberReleases();
            PartialRelease.Name = "Partial bending release";//Name of the object
            PartialRelease.rz = true;//Release the rotational degree of freedom about z axis (in element local coordinate system)
            PartialRelease.krz = 10000;//Assign a spring stiffness (units in moment/rotations, for example kNm/rad)
            //Note that the corresponding degree of freedom should be first released in order to define afterwards a partial stiffness constant
            //In case of full release we should have given PartialRelease.krz = 0;

            //Create frame element 2. Note that the proper release object (Partial Releases is passed in the constructor)
            FrameSuperElement el2 = new FrameSuperElement(2, n2, n3, new Geometry.XYZ(0, 4, 1), matConcrete, secBeam30_70, PartialRelease, PartialRelease, false, false);
            LinearLoadCaseForSuperFrameElement lc1 = new LinearLoadCaseForSuperFrameElement("lc1", LoadCaseType.DEAD);
            lc1.UniformLoad.UniformLoadsY.Add(new FrameSuperUniformLoad(0, 1, -10, -10, LoadDefinitionFromStartingNode.Relatively, LoadCordinateSystem.Global));
            el2.LinearLoadCasesList.Add(lc1);
            Model.InputFiniteElements.Add(el2);

            //Create frame element 3
            FrameSuperElement el3 = new FrameSuperElement(3, n4, n3, new Geometry.XYZ(5, 0, 1), matConcrete, secColumn50_50, new MemberReleases(), new MemberReleases(), false, false);
            Model.InputFiniteElements.Add(el3);

            //-------SOLUTION PHASE-------
            Model.Solve();

            //-------OBTAIN RESULTS-------
            double[] Min, Max;

            //Support reactions
            n1.GetReactionsForLoadCase("lc1", out Min, out Max, 0);
            double n1_Rtx_lc1 = Max[0];
            n1.GetReactionsForLoadCase("lc1", out Min, out Max, 0);
            double n1_Rty_lc1 = Max[1];
            n4.GetReactionsForLoadCase("lc1", out Min, out Max, 0);
            double n4_Rtx_lc1 = Max[0];
            n4.GetReactionsForLoadCase("lc1", out Min, out Max, 0);
            double n4_Rty_lc1 = Max[1];

            //Rotations at nodes 2 and 3 (in local node system)
            n2.GetNodalDisplacementsForLoadCase("lc1", out Min, out Max, 0);//negative rotation
            double n2_Rrz_lc1 = Max[5];//negative rotation
            n3.GetNodalDisplacementsForLoadCase("lc1", out Min, out Max, 0);//the same rotation, but positive
            double n3_Rrz_lc1 = Max[5];//the same rotation, but positive
        }
        #endregion

        #region Example 6
        private void Example_6(object sender, EventArgs e)
        {
            //New model definition            
            Model Model = new Model();
            Model.LicenseInfo = LicenseInfo;

            //-------MATERIAL DEFINITION-------

            //Create a new material for concrete
            Material matConcrete = new Material();
            matConcrete.Name = "Concrete";//Material name
            matConcrete.Density = 2.5;//density in mass units/m3, for example tn/m3
            matConcrete.G = 11538461;//shear modulus
            matConcrete.E = 30000000;//elasticity modulus

            //-------SECTIONS DEFINITION-------

            //Create a new beam section of dimensions 30cmx70xm
            FrameElementSection secBeam30_70 = new FrameElementSection();
            secBeam30_70.Name = "Beam30/70";//section name
            secBeam30_70.A = 0.3 * 0.7;//section area
            secBeam30_70.Iy = 0.3 * 0.7 * 0.7 * 0.7 / 12;//inertia moment about local y axis
            secBeam30_70.Iz = 0.7 * 0.3 * 0.3 * 0.3 / 12;//inertia moment about local z axis
            secBeam30_70.It = 4.347e-3;//torsional constant
            secBeam30_70.b = 0.30;//section height
            secBeam30_70.h = 0.70;//section height

            //Create a new beam section of dimensions 50cmx50xm           
            FrameElementSection secColumn50_50 = new FrameElementSection();
            secColumn50_50.Name = "Column50/50";  //section name        
            secColumn50_50.A = 0.5 * 0.5;//section area
            secColumn50_50.Iy = 0.5 * 0.5 * 0.5 * 0.5 / 12;//inertia moment about local y axis
            secColumn50_50.Iz = 0.5 * 0.5 * 0.5 * 0.5 / 12;//inertia moment about local z axis
            secColumn50_50.It = 8.8125e-3;
            secColumn50_50.b = 0.50;//section height
            secColumn50_50.h = 0.50;//section height

            //-------MODEL GEOMETRY AND LOADS DEFINITION-------

            //Create node n1 
            Frame3D.SuperNode n1 = new Frame3D.SuperNode(1, 0, 0, 0);
            n1.dof1constraint = true;//translational constraint in direction x at local system of node
            n1.dof2constraint = true;//translational constraint in direction y at local system of node
            n1.dof3constraint = true;//translational constraint in direction z at local system of node
            n1.dof4constraint = true;//rotational constraint in direction x at local system of node
            n1.dof5constraint = true;//rotational constraint in direction y at local system of node
            n1.dof6constraint = true;//rotational constraint in direction z at local system of node
            Model.InputNodes.Add(n1);

            //Create node n2
            Frame3D.SuperNode n2 = new Frame3D.SuperNode(2, 0, 4, 0);
            Model.InputNodes.Add(n2);

            //Create node n3
            Frame3D.SuperNode n3 = new Frame3D.SuperNode(3, 5, 4, 0);
            Model.InputNodes.Add(n3);

            //Create node n4
            Frame3D.SuperNode n4 = new Frame3D.SuperNode(4, 5, 0, 0);
            n4.dof1constraint = true;//translational constraint in direction x at local system of node
            n4.dof2constraint = true;//translational constraint in direction y at local system of node
            n4.dof3constraint = true;//translational constraint in direction z at local system of node
            n4.dof4constraint = true;//rotational constraint in direction x at local system of node
            n4.dof5constraint = true;//rotational constraint in direction y at local system of node
            n4.dof6constraint = true;//rotational constraint in direction z at local system of node
            Model.InputNodes.Add(n4);

            //Create frame element 1 
            FrameSuperElement el1 = new FrameSuperElement(1, n1, n2, new Geometry.XYZ(0, 0, 1), matConcrete, secColumn50_50, new MemberReleases(), new MemberReleases(), false, false);
            el1.RigidOffsetEndDx = 0.35;
            Model.InputFiniteElements.Add(el1);

            //Create frame element 2. Note that the proper release object (Partial Releases is passed in the constructor)
            FrameSuperElement el2 = new FrameSuperElement(2, n2, n3, new Geometry.XYZ(0, 4, 1), matConcrete, secBeam30_70, new MemberReleases(), new MemberReleases(), false, false);
            el2.RigidOffsetStartDx = 0.25;
            el2.RigidOffsetEndDx = 0.25;
            LinearLoadCaseForSuperFrameElement lc1 = new LinearLoadCaseForSuperFrameElement("lc1", LoadCaseType.DEAD);
            lc1.UniformLoad.UniformLoadsY.Add(new FrameSuperUniformLoad(0, 1, -10, -10, LoadDefinitionFromStartingNode.Relatively, LoadCordinateSystem.Global));
            el2.LinearLoadCasesList.Add(lc1);
            Model.InputFiniteElements.Add(el2);

            //Create frame element 3
            FrameSuperElement el3 = new FrameSuperElement(3, n4, n3, new Geometry.XYZ(5, 0, 1), matConcrete, secColumn50_50, new MemberReleases(), new MemberReleases(), false, false);
            el3.RigidOffsetEndDx = 0.35;
            Model.InputFiniteElements.Add(el3);

            //-------SOLUTION PHASE-------
            Model.Solve();

            //-------OBTAIN RESULTS-------
            double[] Min, Max;

            //Support reactions
            n1.GetReactionsForLoadCase("lc1", out Min, out Max, 0);
            double n1_Rtx_lc1 = Max[0];
            n1.GetReactionsForLoadCase("lc1", out Min, out Max, 0);
            double n1_Rty_lc1 = Max[1];
            n4.GetReactionsForLoadCase("lc1", out Min, out Max, 0);
            double n4_Rtx_lc1 = Max[0];
            n4.GetReactionsForLoadCase("lc1", out Min, out Max, 0);
            double n4_Rty_lc1 = Max[1];

            //Rotations at nodes 2 and 3 (in local node system)
            n2.GetNodalDisplacementsForLoadCase("lc1", out Min, out Max, 0); //negative rotation
            double n2_Rrz_lc1 = Max[5];//negative rotation
            n3.GetNodalDisplacementsForLoadCase("lc1", out Min, out Max, 0); ;//the same rotation, but positive
            double n3_Rrz_lc1 = Max[5];//the same rotation, but positive
        }
        #endregion

        #region Example 7
        private void Example_7(object sender, EventArgs e)
        {
            //New model definition            
            Model Model = new Model();
            Model.LicenseInfo = LicenseInfo;

            //-------MATERIAL DEFINITION-------

            //Create a new material for concrete
            Material matConcrete = new Material();
            matConcrete.Name = "Concrete";//Material name
            matConcrete.Density = 2.5;//density in mass units/m3, for example tn/m3
            matConcrete.G = 11538461;//shear modulus
            matConcrete.E = 30000000;//elasticity modulus

            //-------SECTIONS DEFINITION-------

            //Create a new beam section of dimensions 30cmx70xm
            FrameElementSection secBeam30_70 = new FrameElementSection();
            secBeam30_70.Name = "Beam30/70";//section name
            secBeam30_70.A = 0.3 * 0.7;//section area
            secBeam30_70.Iy = 0.3 * 0.7 * 0.7 * 0.7 / 12;//inertia moment about local y axis
            secBeam30_70.Iz = 0.8 * 0.3 * 0.3 * 0.3 / 12;//inertia moment about local z axis
            secBeam30_70.It = 4.347e-3;//torsional constant
            secBeam30_70.b = 0.30;//section height
            secBeam30_70.h = 0.70;//section height

            //Create a new beam section of dimensions 50cmx50xm           
            FrameElementSection secColumn50_50 = new FrameElementSection();
            secColumn50_50.Name = "Column50/50";  //section name        
            secColumn50_50.A = 0.5 * 0.5;//section area
            secColumn50_50.Iy = 0.5 * 0.5 * 0.5 * 0.5 / 12;//inertia moment about local y axis
            secColumn50_50.Iz = 0.5 * 0.5 * 0.5 * 0.5 / 12;//inertia moment about local z axis
            secColumn50_50.It = 8.8125e-3;
            secColumn50_50.b = 0.50;//section height
            secColumn50_50.h = 0.50;//section height

            //-------MODEL GEOMETRY AND LOADS DEFINITION-------

            //Create node n1 
            Frame3D.SuperNode n1 = new Frame3D.SuperNode(1, 0, 0, 0);
            n1.dof1constraint = true;//translational constraint in direction x at local system of node
            n1.dof2constraint = true;//translational constraint in direction y at local system of node
            n1.dof3constraint = true;//translational constraint in direction z at local system of node
            n1.dof4constraint = true;//rotational constraint in direction x at local system of node
            n1.dof5constraint = true;//rotational constraint in direction y at local system of node
            n1.dof6constraint = true;//rotational constraint in direction z at local system of node
            Model.InputNodes.Add(n1);

            //Create node n2 
            Frame3D.SuperNode n2 = new Frame3D.SuperNode(2, 5, 0, 0);
            n2.dof1constraint = true;//translational constraint in direction x at local system of node
            n2.dof2constraint = true;//translational constraint in direction y at local system of node
            n2.dof3constraint = true;//translational constraint in direction z at local system of node
            n2.dof4constraint = true;//rotational constraint in direction x at local system of node
            n2.dof5constraint = true;//rotational constraint in direction y at local system of node
            n2.dof6constraint = true;//rotational constraint in direction z at local system of node
            Model.InputNodes.Add(n2);

            //Create node n3
            Frame3D.SuperNode n3 = new Frame3D.SuperNode(3, 0, 6, 0);
            n3.dof1constraint = true;//translational constraint in direction x at local system of node
            n3.dof2constraint = true;//translational constraint in direction y at local system of node
            n3.dof3constraint = true;//translational constraint in direction z at local system of node
            n3.dof4constraint = true;//rotational constraint in direction x at local system of node
            n3.dof5constraint = true;//rotational constraint in direction y at local system of node
            n3.dof6constraint = true;//rotational constraint in direction z at local system of node
            Model.InputNodes.Add(n3);

            //Create node n4
            Frame3D.SuperNode n4 = new Frame3D.SuperNode(4, 5, 6, 0);
            n4.dof1constraint = true;//translational constraint in direction x at local system of node
            n4.dof2constraint = true;//translational constraint in direction y at local system of node
            n4.dof3constraint = true;//translational constraint in direction z at local system of node
            n4.dof4constraint = true;//rotational constraint in direction x at local system of node
            n4.dof5constraint = true;//rotational constraint in direction y at local system of node
            n4.dof6constraint = true;//rotational constraint in direction z at local system of node
            Model.InputNodes.Add(n4);

            //Create node n5
            Frame3D.SuperNode n5 = new Frame3D.SuperNode(5, 0, 0, 3);
            Model.InputNodes.Add(n5);

            //Create node n6
            Frame3D.SuperNode n6 = new Frame3D.SuperNode(6, 5, 0, 3);
            Model.InputNodes.Add(n6);

            //Create node n7
            Frame3D.SuperNode n7 = new Frame3D.SuperNode(7, 0, 6, 3);
            Model.InputNodes.Add(n7);

            //Create node n8
            Frame3D.SuperNode n8 = new Frame3D.SuperNode(8, 5, 6, 3);
            Model.InputNodes.Add(n8);

            //Create node n9
            Frame3D.SuperNode n9 = new Frame3D.SuperNode(9, 0, 0, 6);
            Model.InputNodes.Add(n9);

            //Create node n10
            Frame3D.SuperNode n10 = new Frame3D.SuperNode(10, 5, 0, 6);
            Model.InputNodes.Add(n10);

            //Create node n11
            Frame3D.SuperNode n11 = new Frame3D.SuperNode(11, 0, 6, 6);
            Model.InputNodes.Add(n11);

            //Create node n12
            Frame3D.SuperNode n12 = new Frame3D.SuperNode(12, 5, 6, 6);
            Model.InputNodes.Add(n12);


            //Create frame elements (Note the definition of the auxiliary point which is different for each frame in order to correclty place it
            //It is reminded that auxiliary point is only only used to define the rotation of the frame element about its longitudinal axis
            //This point should not belong to the longitudinal axis of the element. In such case, arithmetic errors would occur.


            //Create first story columns

            FrameSuperElement el1 = new FrameSuperElement(1, n1, n5, new Geometry.XYZ(0, 1, 0), matConcrete, secColumn50_50, new MemberReleases(), new MemberReleases(), false, false);
            Model.InputFiniteElements.Add(el1);
            FrameSuperElement el2 = new FrameSuperElement(2, n2, n6, new Geometry.XYZ(5, 1, 0), matConcrete, secColumn50_50, new MemberReleases(), new MemberReleases(), false, false);
            Model.InputFiniteElements.Add(el2);
            FrameSuperElement el3 = new FrameSuperElement(3, n4, n8, new Geometry.XYZ(5, 7, 0), matConcrete, secColumn50_50, new MemberReleases(), new MemberReleases(), false, false);
            Model.InputFiniteElements.Add(el3);
            FrameSuperElement el4 = new FrameSuperElement(4, n3, n7, new Geometry.XYZ(0, 7, 0), matConcrete, secColumn50_50, new MemberReleases(), new MemberReleases(), false, false);
            Model.InputFiniteElements.Add(el4);


            //Create first story beams

            FrameSuperElement el5 = new FrameSuperElement(5, n5, n6, new Geometry.XYZ(0, 1, 3), matConcrete, secColumn50_50, new MemberReleases(), new MemberReleases(), false, false);
            Model.InputFiniteElements.Add(el5);
            FrameSuperElement el6 = new FrameSuperElement(6, n6, n8, new Geometry.XYZ(4, 0, 3), matConcrete, secColumn50_50, new MemberReleases(), new MemberReleases(), false, false);
            Model.InputFiniteElements.Add(el6);
            FrameSuperElement el7 = new FrameSuperElement(7, n7, n8, new Geometry.XYZ(0, 7, 3), matConcrete, secColumn50_50, new MemberReleases(), new MemberReleases(), false, false);
            Model.InputFiniteElements.Add(el7);
            FrameSuperElement el8 = new FrameSuperElement(8, n5, n7, new Geometry.XYZ(-1, 0, 3), matConcrete, secColumn50_50, new MemberReleases(), new MemberReleases(), false, false);
            Model.InputFiniteElements.Add(el8);

            //Create second story columns

            FrameSuperElement el13 = new FrameSuperElement(13, n9, n10, new Geometry.XYZ(0, 1, 3), matConcrete, secColumn50_50, new MemberReleases(), new MemberReleases(), false, false);
            Model.InputFiniteElements.Add(el13);
            FrameSuperElement el14 = new FrameSuperElement(14, n10, n12, new Geometry.XYZ(4, 0, 3), matConcrete, secColumn50_50, new MemberReleases(), new MemberReleases(), false, false);
            Model.InputFiniteElements.Add(el14);
            FrameSuperElement el15 = new FrameSuperElement(15, n11, n12, new Geometry.XYZ(0, 7, 3), matConcrete, secColumn50_50, new MemberReleases(), new MemberReleases(), false, false);
            Model.InputFiniteElements.Add(el15);
            FrameSuperElement el16 = new FrameSuperElement(16, n9, n11, new Geometry.XYZ(-1, 0, 3), matConcrete, secColumn50_50, new MemberReleases(), new MemberReleases(), false, false);
            Model.InputFiniteElements.Add(el16);

            //Create second story beams

            FrameSuperElement el9 = new FrameSuperElement(9, n5, n9, new Geometry.XYZ(0, 1, 3), matConcrete, secColumn50_50, new MemberReleases(), new MemberReleases(), false, false);
            Model.InputFiniteElements.Add(el9);
            FrameSuperElement el10 = new FrameSuperElement(10, n6, n10, new Geometry.XYZ(5, 1, 3), matConcrete, secColumn50_50, new MemberReleases(), new MemberReleases(), false, false);
            Model.InputFiniteElements.Add(el10);
            FrameSuperElement el11 = new FrameSuperElement(11, n8, n12, new Geometry.XYZ(5, 7, 3), matConcrete, secColumn50_50, new MemberReleases(), new MemberReleases(), false, false);
            Model.InputFiniteElements.Add(el11);
            FrameSuperElement el12 = new FrameSuperElement(12, n7, n11, new Geometry.XYZ(0, 7, 3), matConcrete, secColumn50_50, new MemberReleases(), new MemberReleases(), false, false);
            Model.InputFiniteElements.Add(el12);

            //Create a list of Geometry.XY objects with the boundary points of the floor diaphragms
            //A polygon is then internally defined and all points that liew it it or on its edges will be assumed to be restarined by the diaphragm
            List<Geometry.XY> Pts = new List<Geometry.XY>();
            //Points should be given anti-clockwise
            //Points are given in plan view (x-y)
            //Points (if more than 3) should lie on the same plane
            Pts.Add(new Geometry.XY(0, 0));
            Pts.Add(new Geometry.XY(5, 0));
            Pts.Add(new Geometry.XY(5, 6));
            Pts.Add(new Geometry.XY(0, 6));

            //Floor diaphragm definition. Note that the 3rd argument specifies the z-coordinate of diaphragm
            //Floor diaphragm is defined in xy plane only. Global Z axis is always perpendicular to the plane that the diaphragm points define
            FloorDiaphragm fd1 = new FloorDiaphragm(1, Pts, 3);
            //Create a load case than specifies the mass source for the diaphragm
            //This load case only defines the mass for the diaphragm and is only needed in dynamic analysis.
            //Vertical static loads are not taken into account from this load case.
            //If a diaphragm is loaded, the corresponding mass load case should be assigned. Then the diaphragm mass will be considered for the dynamic analysis
            //A load case for the perimetric beams should then manually be created, which will distribute the diaphragm loads to the frames. This is not made automatically by the library
            LinearLoadCaseForFloorDiaphragm mass_fd1 = new LinearLoadCaseForFloorDiaphragm("mass source", LoadCaseType.DEAD);
            mass_fd1.pz = 5;//units in force/area, for example kN/m2, positive direction = gravity
            fd1.LinearLoadCasesList.Add(mass_fd1);
            Model.FloorDiaphragms.Add(fd1);
            //Similarily create a floor diaphragm for second story
            FloorDiaphragm fd2 = new FloorDiaphragm(2, Pts, 6);
            LinearLoadCaseForFloorDiaphragm mass_fd2 = new LinearLoadCaseForFloorDiaphragm("mass source", LoadCaseType.DEAD);
            mass_fd2.pz = 2;//units in force/area, for example kN/m2, positive direction = gravity
            fd2.LinearLoadCasesList.Add(mass_fd2);
            Model.FloorDiaphragms.Add(fd2);

            //Define a load combination for the mass for the diaphragms (for example DEAD+0.5LIVE etc)
            LoadCombination MassCombo = new LoadCombination("mass combo", ComboType.ADD);
            MassCombo.Items.Add(new LoadCaseWithFactor("mass source", 1.0));
            Model.MassSourceCombination = MassCombo;

            //Specify how mass is going to be calculated
            GeneralData.IncludeAdditionalMassesInMassSource = true;
            GeneralData.IncludeLoadsInMassSource = true;
            GeneralData.IncludeSelfWeightInMassSource = true;

            //Create a response spectrum function
            ResponseSpectrumFunction RSFunction = new ResponseSpectrumFunction("RS function");
            RSFunction.RS_T = new double[] { 0, 0.15, 0.50, 1.20 };//T (time) values of point of the spectrum (in sec)
            RSFunction.RS_A = new double[] { 0, 5.5, 5.5, 1.0 };//A (spectral acceleration) values of points in spectrum (in length/sec2, for example m/sec2) 

            //Create a response spectrum case and specify the application direction and the modal combination rule (SRSS or CQC)
            ResponseSpectrumCase RSCase = new ResponseSpectrumCase("RScase", GroundMotionDirection.UX, ModeComboType.CQC);
            RSCase.DiaphragmEccentricityRatio = 0.05;//Specify diaphragm eccentricity ratio (usually 5%-10%). This value will produce a torsional about the global Z coordinate at the center of mass of each diaphragm.
            RSCase.RSFunction = RSFunction;//Assign the previously defined response spectrum
            Model.ResponseSpectrumCases.Add(RSCase);//Add to model

            Model.NrOfModesToFind = 6;

            //-------SOLUTION PHASE-------
            Model.Solve();

            //-------OBTAIN RESULTS-------
            //Effective mass ratio calculation:
            double Effmx = Model.TotalEffectiveMassUX;//mass excited in x direction
            double Effmy = Model.TotalEffectiveMassUY;//mass excited in y direction
            double Massmx = Model.TotalMassUX;//total lateral mass in x direction
            double Massmy = Model.TotalMassUY;//total lateral mass in y direction
            double ratio_mass_x = Effmx / Massmx;//>90% of the total mass is excited by the response spectrum analysis
            double ratio_mass_y = Effmy / Massmy;//>90% of the total mass is excited by the response spectrum analysis

            //Reactions (Note that all results are now envelopes beacuse they came from a dynamic analysis)
            double[] Min1, Max1;
            double[] Min2, Max2;
            double[] Min3, Max3;
            double[] Min4, Max4;
            n1.GetReactionsForLoadCase(RSCase.name, out Min1, out Max1, 0);
            n2.GetReactionsForLoadCase(RSCase.name, out Min2, out Max2, 0);
            n3.GetReactionsForLoadCase(RSCase.name, out Min3, out Max3, 0);
            n4.GetReactionsForLoadCase(RSCase.name, out Min4, out Max4, 0);

            //Modal information
            double[,] Modes = Model.Modes;//each rows represents each degree of freedom, each column represents the corresponding modal displacements

            //Periods
            double[] Periods = Model.Periods;//each entry represents the period of the corresponding node

            //Element 2 (el2) internal forces for response spectrum case
            double[] Min, Max;
            el2.GetInternalForcesForLoadCase(0, "RScase", out Min, out Max, 0);
        }
        #endregion

        #region Example 8
        private void Example_8(object sender, EventArgs e)
        {
            //New model definition            
            Model Model = new Model();
            Model.LicenseInfo = LicenseInfo;

            //-------MATERIAL DEFINITION-------

            //Create a new material for concrete
            Material matConcrete = new Material();
            matConcrete.Name = "Concrete";//Material name
            matConcrete.Density = 2.5;//density in mass units/m3, for example tn/m3
            matConcrete.G = 11538461;//shear modulus
            matConcrete.E = 30000000;//elasticity modulus

            //-------SECTIONS DEFINITION-------

            //Create a new beam section of dimensions 50cmx50xm
            FrameElementSection secBeam50_50 = new FrameElementSection();
            secBeam50_50.Name = "Beam50/50";//section name
            secBeam50_50.A = 0.5 * 0.5;//section area
            secBeam50_50.Iy = 0.5 * 0.5 * 0.5 * 0.5 / 12;//inertia moment about local y axis
            secBeam50_50.Iz = 0.5 * 0.5 * 0.5 * 0.5 / 12;//inertia moment about local z axis
            secBeam50_50.It = 4.347e-3;//torsional constant
            secBeam50_50.b = 0.5;//section height
            secBeam50_50.h = 0.5;//section height

            //-------MODEL GEOMETRY AND LOADS DEFINITION-------

            //First node creation
            Frame3D.SuperNode n1 = new Frame3D.SuperNode(1, 0, 0, 0);
            //Application of supports (fixed conditions out of plane)
            n1.dof1constraint = true;
            n1.dof2constraint = true;
            n1.dof3constraint = true;
            n1.dof4constraint = true;
            n1.dof5constraint = false;
            n1.dof6constraint = true;
            Model.InputNodes.Add(n1);

            //Second node creation
            Frame3D.SuperNode n2 = new Frame3D.SuperNode(2, 5, 0, 0);
            //Application of supports (fixed conditions out of plane)
            n2.dof1constraint = false;
            n2.dof2constraint = true;
            n2.dof3constraint = true;
            n2.dof4constraint = true;
            n2.dof5constraint = false;
            n2.dof6constraint = true;
            //Load case creation for horizontal load acting at right node
            LinearLoadCaseForSuperNode L = new LinearLoadCaseForSuperNode("L", LoadCaseType.OTHER);
            L.Px = -1000;
            n2.LinearLoadCasesList.Add(L);
            Model.InputNodes.Add(n2);

            //Frame element creation
            FrameSuperElement el1 = new FrameSuperElement(1, n1, n2, new Geometry.XYZ(0, 1, 0), matConcrete, secBeam50_50, new MemberReleases(), new MemberReleases(), false, false);
            //Load case creation for uniform vertical load on frame element
            LinearLoadCaseForSuperFrameElement load1 = new LinearLoadCaseForSuperFrameElement("L", LoadCaseType.OTHER);
            load1.UniformLoad.UniformLoadsZ.Add(new FrameSuperUniformLoad(0, 1, -10, -10, LoadDefinitionFromStartingNode.Relatively, LoadCordinateSystem.Local));
            el1.LinearLoadCasesList.Add(load1);
            Model.InputFiniteElements.Add(el1);

            //Creation of a geometric non linear case that includes all load cases defined as "L"
            GeometricNonLinearCase NLcase = new GeometricNonLinearCase("NL");
            //Analysis parameters:
            NLcase.LoadSteps = 50;//50 load steps
            NLcase.IterationsPerLoadStep = 30;//maximum 30 iteration per load step
            NLcase.ConvergenceTolerance = 1e-12;//convergence tolerance in terms of force
            //It will include the loads that have been defined as "L"
            NLcase.InputLoadCombo = new LoadCombination("NLcase loads", ComboType.ADD);
            NLcase.InputLoadCombo.Items.Add(new LoadCaseWithFactor("L", 1));
            //Definition of stiffness matrix update mode
            NLcase.UpdateStiffnessMethod = GeometricNonLinearCase.UpdateStiffnessMatrixMethod.AfterEachIterationInLoadStep;
            NLcase.SaveResultsAtEachLoadStep = true;//Results will be saved at all intermediate load steps
            Model.GeometricNonLinearCases.Add(NLcase);

            //-------SOLUTION PHASE-------
            el1.Section.StiffnessModifiers.AMod = 0.1;

            Model.Solve();

            //-------OBTAIN RESULTS-------
            double[] Min, Max;
            for (int loadStep = 1; loadStep <= NLcase.LoadSteps; loadStep++)
            {
                //Get deflection at the middle of the beam at each load step
                el1.GetDisplacementsForLoadCase(2.5, "NL", out Min, out Max, loadStep);
                double Deflection = Min[2];

                //Get bending moment at the middle of the beam at each load step
                el1.GetInternalForcesForLoadCase(2.5, "NL", out Min, out Max, loadStep);
                double SpanMoment = Min[4];
            }
        }
        #endregion

        #region Example 9
        private void Example_9(object sender, EventArgs e)
        {
            //New model definition            
            Model Model = new Model();
            Model.LicenseInfo = LicenseInfo;

            //-------MATERIAL DEFINITION-------

            //Create a new material for concrete
            Material matConcrete = new Material();
            matConcrete.Name = "Concrete";//Material name
            matConcrete.Density = 2.5;//density in mass units/m3, for example tn/m3
            matConcrete.G = 11538461;//shear modulus
            matConcrete.E = 30000000;//elasticity modulus

            //-------SECTIONS DEFINITION-------

            //Create a new column section of dimensions 50cmx50xm
            FrameElementSection secCol050_50 = new FrameElementSection();
            secCol050_50.Name = "Column50/50";//section name
            secCol050_50.A = 0.5 * 0.5;//section area
            secCol050_50.Iy = 0.5 * 0.5 * 0.5 * 0.5 / 12;//inertia moment about local y axis
            secCol050_50.Iz = 0.5 * 0.5 * 0.5 * 0.5 / 12;//inertia moment about local z axis
            secCol050_50.It = 4.347e-3;//torsional constant
            secCol050_50.b = 0.5;//section height
            secCol050_50.h = 0.5;//section height

            //-------MODEL GEOMETRY AND LOADS DEFINITION-------

            //First node creation
            Frame3D.SuperNode n1 = new Frame3D.SuperNode(1, 0, 0, 0);
            //Application of supports (fixed conditions out of plane)
            n1.dof1constraint = true;
            n1.dof2constraint = true;
            n1.dof3constraint = true;
            n1.dof4constraint = true;
            n1.dof5constraint = true;
            n1.dof6constraint = true;
            Model.InputNodes.Add(n1);

            //Second node creation
            Frame3D.SuperNode n2 = new Frame3D.SuperNode(2, 0, 0, 5);
            //Application of supports (fixed conditions out of plane)
            n2.dof1constraint = false;
            n2.dof2constraint = true;
            n2.dof3constraint = false;
            n2.dof4constraint = true;
            n2.dof5constraint = false;
            n2.dof6constraint = true;

            //Load case creation for horizontal and vertical load acting at top node
            LinearLoadCaseForSuperNode L = new LinearLoadCaseForSuperNode("L", LoadCaseType.OTHER);
            L.Px = 100;
            L.Pz = -1000;
            n2.LinearLoadCasesList.Add(L);
            Model.InputNodes.Add(n2);

            FrameSuperElement el1 = new FrameSuperElement(1, n1, n2, new Geometry.XYZ(0, 1, 0), matConcrete, secCol050_50, new MemberReleases(), new MemberReleases(), false, false);

            Model.InputFiniteElements.Add(el1);

            //Creation of a geometric non linear case
            GeometricNonLinearCase NLcase = new GeometricNonLinearCase("NL");
            //Analysis parameters:
            NLcase.LoadSteps = 50;//50 load steps
            NLcase.IterationsPerLoadStep = 30;//maximum 30 iteration per load step
            NLcase.ConvergenceTolerance = 1e-12;//convergence tolerance in terms of force
            //It will include the loads that have been defined as "L"
            NLcase.InputLoadCombo = new LoadCombination("NLcase loads", ComboType.ADD);
            NLcase.InputLoadCombo.Items.Add(new LoadCaseWithFactor("L", 1));
            //Definition of stiffness matrix update mode
            NLcase.UpdateStiffnessMethod = GeometricNonLinearCase.UpdateStiffnessMatrixMethod.AfterEachIterationInLoadStep;
            NLcase.SaveResultsAtEachLoadStep = true;//Results will be saved at all intermediate load steps
            Model.GeometricNonLinearCases.Add(NLcase);

            //-------SOLUTION PHASE-------
            Model.Solve();

            //-------OBTAIN RESULTS-------
            double[] Min, Max;
            for (int loadStep = 1; loadStep <= NLcase.LoadSteps; loadStep++)
            {
                //Get horizontal displacement of top node of the column at each load step             
                n2.GetNodalDisplacementsForLoadCase("NL", out Min, out Max, loadStep);
                double horDisplacement = Min[0];

                //Get bending moment at the base of the column at each load step
                el1.GetInternalForcesForLoadCase(0, "NL", out Min, out Max, loadStep);
                double BaseMoment = Min[4];
            }
        }
        #endregion

        #region Example 10
        private void Example_10(object sender, EventArgs e)
        {
            //New model definition            
            Model Model = new Model();
            Model.LicenseInfo = LicenseInfo;

            //-------MATERIAL DEFINITION-------

            //Create a new material for concrete
            Material matConcrete = new Material();
            matConcrete.Name = "Concrete";//Material name
            matConcrete.Density = 2.5;//density in mass units/m3, for example tn/m3
            matConcrete.G = 11538461;//shear modulus
            matConcrete.E = 30000000;//elasticity modulus

            //-------SECTIONS DEFINITION-------

            //Create a new beam section of dimensions 40cmx80xm
            FrameElementSection secBeam40_80 = new FrameElementSection();
            secBeam40_80.Name = "Beam40/80";//section name
            secBeam40_80.A = 0.4 * 0.8;//section area
            secBeam40_80.Iy = 0.4 * 0.8 * 0.8 * 0.8 / 12;//inertia moment about local y axis
            secBeam40_80.Iz = 0.8 * 0.4 * 0.4 * 0.4 / 12;//inertia moment about local z axis
            secBeam40_80.It = 0.0117248;//torsional constant
            secBeam40_80.b = 0.40;//section width
            secBeam40_80.h = 0.80;//section height

            //-------MODEL GEOMETRY AND LOADS DEFINITION-------

            //Create node n1
            Frame3D.SuperNode n1 = new Frame3D.SuperNode(1, 0, 0, 0);
            n1.dof1constraint = true;//delete
            n1.dof2constraint = true;//translational constraint in direction y at local system of node
            n1.dof3constraint = true;//translational constraint in direction z at local system of node
            n1.dof4constraint = true;//rotational constraint in direction x at local system of node
            n1.dof5constraint = true;//rotational constraint in direction y at local system of node
            Model.InputNodes.Add(n1);

            //Create node n2            
            Frame3D.SuperNode n2 = new Frame3D.SuperNode(2, 5, 0, 0);
            n2.dof1constraint = true;//translational constraint in direction x at local system of node
            n2.dof2constraint = true;//translational constraint in direction y at local system of node
            n2.dof3constraint = true;//translational constraint in direction z at local system of node
            n2.dof4constraint = true;//rotational constraint in direction x at local system of node
            n2.dof5constraint = true;//rotational constraint in direction y at local system of node
            Model.InputNodes.Add(n2);

            //Create frame element 1
            //Note that the 4th argument specifies the auxiliary point that lies in the xy plane that is formed by the x and y axes in the local element system
            FrameSuperElement el1 = new FrameSuperElement(1, n1, n2, new Geometry.XYZ(0, 0, 1), matConcrete, secBeam40_80, new MemberReleases(), new MemberReleases(), false, false);
            el1.WinklerStiffnessZ = 15000;//Winkler spring constant on local Z frame axis (units: force/length/length)

            LinearLoadCaseForSuperFrameElement lc1 = new LinearLoadCaseForSuperFrameElement("lc1", LoadCaseType.DEAD);
            lc1.UniformLoad.UniformLoadsY.Add(new FrameSuperUniformLoad(0, 1, -10, -10, LoadDefinitionFromStartingNode.Relatively, LoadCordinateSystem.Global));
            lc1.PointLoad.PointLoadsY.Add(new SuperPointLoad(3.5, -50, LoadDefinitionFromStartingNode.Absolutely, LoadCordinateSystem.Global));
            el1.LinearLoadCasesList.Add(lc1);

            Model.InputFiniteElements.Add(el1);

            //-------SOLUTION PHASE-------
            Model.Solve();

            //-------OBTAIN RESULTS-------
            double[] Min, Max;//The combination results will be saved in these arrays
            //Note that the definition of two arrays for minimum and maximum combination results is required
            //For combination type "ADD", Min and Max values are always equal

            //Reactions (All are defined in the node local system)
            //Rections for load case lc1
            n1.GetReactionsForLoadCase("lc1", out Min, out Max, 0);
            double n1_Rty_lc1 = Max[1];
            n2.GetReactionsForLoadCase("lc1", out Min, out Max, 0);
            double n2_Rty_lc1 = Max[1];

            //Node Displacements (All are defined in the node local system)
            //Note that constained degrees of freedom have zero displacements
            n1.GetNodalDisplacementsForLoadCase("lc1", out Min, out Max, 0);
            double[] n1_Disp = Max;
            n2.GetNodalDisplacementsForLoadCase("lc1", out Min, out Max, 0);
            double[] n2_Disp = Max;

            //Element internal forces and displacements
            el1.GetInternalForcesForLoadCase(0, "lc1", out Min, out Max, 0); //Internal forces at the start of the member
            double[] forces_along_member_left = Max;
            el1.GetInternalForcesForLoadCase(2.5, "lc1", out Min, out Max, 0);//Internal forces at the middle of the member
            double[] forces_along_member_middle = Max;
            el1.GetInternalForcesForLoadCase(5, "lc1", out Min, out Max, 0);//Internal forces at the end of the member
            double[] forces_along_member_right = Max;

            el1.GetDisplacementsForLoadCase(0, "lc1", out Min, out Max, 0); //Internal displacements at the start of the member
            double[] disps_along_member_left = Max;
            el1.GetDisplacementsForLoadCase(2.5, "lc1", out Min, out Max, 0);//Internal displacements at the middle of the member
            double[] disps_along_member_middle = Max;
            el1.GetDisplacementsForLoadCase(5, "lc1", out Min, out Max, 0);//Internal displacements at the end of the member
            double[] disps_along_member_right = Max;

            el1.GetWinklerSpringReactionsForLoadCase(0, "lc1", out Min, out Max, 0); //Soil reaction at the start of the member (units in force: length)
            double[] soil_reaction_left = Max;
            el1.GetWinklerSpringReactionsForLoadCase(2.5, "lc1", out Min, out Max, 0); //Soil reaction at the middle of the member (units in force: length)
            double[] soil_reaction_middle = Max;
            el1.GetWinklerSpringReactionsForLoadCase(5, "lc1", out Min, out Max, 0); //Soil reaction at the end of the member (units in force: length)
            double[] soil_reaction_right = Max;
        }
        #endregion

        #region Example 11
        private void Example_11(object sender, EventArgs e)
        {
            //New model definition            
            Model Model = new Model();
            Model.LicenseInfo = LicenseInfo;

            //Create a new material for concrete
            Material matConcrete = new Material();
            matConcrete.Name = "Concrete";//Material name
            matConcrete.Density = 2.5;//density in mass units/m3, for example tn/m3
            matConcrete.G = 30000000 / 2.6;//shear modulus
            matConcrete.E = 30000000;//elasticity modulus

            //Create a cross section for frames
            FrameElementSection ConcreteSection = new FrameElementSection();
            ConcreteSection.Name = "Concrete section";//section name
            ConcreteSection.A = 0.4 * 0.8;//section area
            ConcreteSection.Iy = 0.4 * 0.8 * 0.8 * 0.8 / 12;//inertia moment about local y axis
            ConcreteSection.Iz = 0.8 * 0.4 * 0.4 * 0.4 / 12;//inertia moment about local z axis
            ConcreteSection.It = 0.0117248;//torsional constant
            ConcreteSection.b = 0.40;//section height
            ConcreteSection.h = 0.80;//section height

            Frame3D.SuperNode n1, n2, n3, n4;
            ShellSuperElementTriangular ssel1;
            ShellSuperElementTriangular ssel2;

            double soilStiffnessPerArea = 5000;

            int i_nodes = 1;
            //Create foundation nodes (these nodes will be used to create the shell elements of the foundation slab)
            for (double x = 0; x <= 7; x += 1.0)
            {
                for (double y = 0; y <= 8; y += 1.0)
                {
                    Frame3D.SuperNode n = new Frame3D.SuperNode(i_nodes++, x, y, 0);
                    n.dof1constraint = true;
                    n.dof2constraint = true;
                    n.Kdof3 = soilStiffnessPerArea;
                    n.dof6constraint = true;
                    Model.InputNodes.Add(n);
                }
            }

            //The foundation slab elements are created below. The thickness of the foundation is 0.5 m.
            int i_elements = 1;
            for (double y = 1; y <= 8; y += 1.0)
            {
                for (double x = 1; x <= 7; x += 1.0)
                {
                    double y12 = y - 1;
                    double y34 = y;

                    double x14 = x - 1;
                    double x23 = x;

                    n1 = Model.InputNodes.Find(p => p.x == x14 && p.y == y12 && p.z == 0);
                    n2 = Model.InputNodes.Find(p => p.x == x23 && p.y == y12 && p.z == 0);
                    n3 = Model.InputNodes.Find(p => p.x == x23 && p.y == y34 && p.z == 0);
                    n4 = Model.InputNodes.Find(p => p.x == x14 && p.y == y34 && p.z == 0);

                    //ssel = new ShellSuperElement(i_elements++, n1, n2, n3, n4, matConcrete);
                    ssel1 = new ShellSuperElementTriangular(i_elements++, n1, n2, n3, matConcrete);
                    ssel1.Thickness = 0.5;
                    ssel1.Spring_Z_local_Stiffness_per_Area = soilStiffnessPerArea;
                    Model.InputFiniteElements.Add(ssel1);
                    ssel2 = new ShellSuperElementTriangular(i_elements++, n1, n3, n4, matConcrete);
                    ssel2.Thickness = 0.5;
                    ssel2.Spring_Z_local_Stiffness_per_Area = soilStiffnessPerArea;
                    Model.InputFiniteElements.Add(ssel2);
                }
            }

            //The nodes of the story are created
            Frame3D.SuperNode ns1 = new Frame3D.SuperNode(i_nodes++, 1, 1, 3);
            Model.InputNodes.Add(ns1);

            Frame3D.SuperNode ns2 = new Frame3D.SuperNode(i_nodes++, 6, 1, 3);
            Model.InputNodes.Add(ns2);

            Frame3D.SuperNode ns3 = new Frame3D.SuperNode(i_nodes++, 6, 7, 3);
            Model.InputNodes.Add(ns3);

            Frame3D.SuperNode ns4 = new Frame3D.SuperNode(i_nodes++, 1, 7, 3);
            Model.InputNodes.Add(ns4);


            //The following two nodes belong to the vertical wall at elevation z=1.00 and z=2.00
            Frame3D.SuperNode nsw1_z1 = new Frame3D.SuperNode(i_nodes++, 2, 1, 1);
            Model.InputNodes.Add(nsw1_z1);
            Frame3D.SuperNode nsw2_z1 = new Frame3D.SuperNode(i_nodes++, 3, 1, 1);
            Model.InputNodes.Add(nsw2_z1);

            Frame3D.SuperNode nsw1_z2 = new Frame3D.SuperNode(i_nodes++, 2, 1, 2);
            Model.InputNodes.Add(nsw1_z2);
            Frame3D.SuperNode nsw2_z2 = new Frame3D.SuperNode(i_nodes++, 3, 1, 2);
            Model.InputNodes.Add(nsw2_z2);


            //The shell elements of the wall are created. (the wall has a thickness of 0.25 m)
            n1 = Model.InputNodes.Find(p => p.x == 2 && p.y == 1 && p.z == 0);
            n2 = Model.InputNodes.Find(p => p.x == 3 && p.y == 1 && p.z == 0);
            n3 = Model.InputNodes.Find(p => p.x == 2 && p.y == 1 && p.z == 1);
            n4 = Model.InputNodes.Find(p => p.x == 3 && p.y == 1 && p.z == 1);
            ssel1 = new ShellSuperElementTriangular(i_elements++, n1, n2, n3, matConcrete);
            ssel1.Thickness = 0.25;
            Model.InputFiniteElements.Add(ssel1);
            ssel2 = new ShellSuperElementTriangular(i_elements++, n1, n3, n4, matConcrete);
            ssel2.Thickness = 0.25;
            Model.InputFiniteElements.Add(ssel2);

            n1 = Model.InputNodes.Find(p => p.x == 2 && p.y == 1 && p.z == 1);
            n2 = Model.InputNodes.Find(p => p.x == 3 && p.y == 1 && p.z == 1);
            n3 = Model.InputNodes.Find(p => p.x == 2 && p.y == 1 && p.z == 2);
            n4 = Model.InputNodes.Find(p => p.x == 3 && p.y == 1 && p.z == 2);
            ssel1 = new ShellSuperElementTriangular(i_elements++, n1, n2, n3, matConcrete);
            ssel1.Thickness = 0.25;
            Model.InputFiniteElements.Add(ssel1);
            ssel2 = new ShellSuperElementTriangular(i_elements++, n1, n3, n4, matConcrete);
            ssel2.Thickness = 0.25;
            Model.InputFiniteElements.Add(ssel2);

            n1 = Model.InputNodes.Find(p => p.x == 2 && p.y == 1 && p.z == 2);
            n2 = Model.InputNodes.Find(p => p.x == 3 && p.y == 1 && p.z == 2);
            //The upper wall joints are created here:
            n3 = new SuperNode(i_nodes++, 2, 1, 3);
            LinearLoadCaseForSuperNode llcsn = new LinearLoadCaseForSuperNode("lc1", LoadCaseType.DEAD);
            llcsn.Pz = -100;
            n3.LinearLoadCasesList.Add(llcsn);

            Model.InputNodes.Add(n3);
            n4 = new SuperNode(i_nodes++, 3, 1, 3);
            Model.InputNodes.Add(n4);
            ssel1 = new ShellSuperElementTriangular(i_elements++, n1, n2, n3, matConcrete);
            ssel1.Thickness = 0.25;
            Model.InputFiniteElements.Add(ssel1);
            ssel2 = new ShellSuperElementTriangular(i_elements++, n1, n3, n4, matConcrete);
            ssel2.Thickness = 0.25;
            Model.InputFiniteElements.Add(ssel2);

            //The columns are created below
            Frame3D.FrameSuperElement c1 = new FrameSuperElement(i_elements++, Model.InputNodes[10], ns1, new Geometry.XYZ(1, 2, 0), matConcrete, ConcreteSection, new MemberReleases(), new MemberReleases(), true, false);
            Model.InputFiniteElements.Add(c1);

            Frame3D.FrameSuperElement c2 = new FrameSuperElement(i_elements++, Model.InputNodes[55], ns2, new Geometry.XYZ(6, 2, 0), matConcrete, ConcreteSection, new MemberReleases(), new MemberReleases(), true, false);
            Model.InputFiniteElements.Add(c2);

            Frame3D.FrameSuperElement c3 = new FrameSuperElement(i_elements++, Model.InputNodes[61], ns3, new Geometry.XYZ(6, 8, 0), matConcrete, ConcreteSection, new MemberReleases(), new MemberReleases(), true, false);
            Model.InputFiniteElements.Add(c3);

            Frame3D.FrameSuperElement c4 = new FrameSuperElement(i_elements++, Model.InputNodes[16], ns4, new Geometry.XYZ(1, 8, 0), matConcrete, ConcreteSection, new MemberReleases(), new MemberReleases(), true, false);
            Model.InputFiniteElements.Add(c4);

            //Create beams
            LinearLoadCaseForSuperFrameElement beamLoad = new LinearLoadCaseForSuperFrameElement("lc1", LoadCaseType.DEAD);
            beamLoad.UniformLoad.UniformLoadsZ.Add(new FrameSuperUniformLoad(0, 1, -10, -10, LoadDefinitionFromStartingNode.Relatively, LoadCordinateSystem.Global));
            //Beam starts from column c1 and ends at column c2. It also intersects the wall. Thus it is modelled using 3 parts.
            Frame3D.FrameSuperElement b1a = new FrameSuperElement(i_elements++, ns1, n3, new Geometry.XYZ(1, 2, 3), matConcrete, ConcreteSection, new MemberReleases(), new MemberReleases(), true, false);
            b1a.LinearLoadCasesList.Add(beamLoad);
            Model.InputFiniteElements.Add(b1a);
            Frame3D.FrameSuperElement b1b = new FrameSuperElement(i_elements++, n3, n4, new Geometry.XYZ(1, 2, 3), matConcrete, ConcreteSection, new MemberReleases(), new MemberReleases(), true, false);
            b1b.LinearLoadCasesList.Add(beamLoad);
            Model.InputFiniteElements.Add(b1b);
            Frame3D.FrameSuperElement b1c = new FrameSuperElement(i_elements++, n4, ns2, new Geometry.XYZ(1, 2, 3), matConcrete, ConcreteSection, new MemberReleases(), new MemberReleases(), true, false);
            b1c.LinearLoadCasesList.Add(beamLoad);
            Model.InputFiniteElements.Add(b1c);

            Frame3D.FrameSuperElement b2 = new FrameSuperElement(i_elements++, ns2, ns3, new Geometry.XYZ(5, 0, 3), matConcrete, ConcreteSection, new MemberReleases(), new MemberReleases(), true, false);
            b2.LinearLoadCasesList.Add(beamLoad);
            Model.InputFiniteElements.Add(b2);

            Frame3D.FrameSuperElement b3 = new FrameSuperElement(i_elements++, ns3, ns4, new Geometry.XYZ(6, 8, 3), matConcrete, ConcreteSection, new MemberReleases(), new MemberReleases(), true, false);
            b3.LinearLoadCasesList.Add(beamLoad);
            Model.InputFiniteElements.Add(b3);

            Frame3D.FrameSuperElement b4 = new FrameSuperElement(i_elements++, ns4, ns1, new Geometry.XYZ(0, 7, 3), matConcrete, ConcreteSection, new MemberReleases(), new MemberReleases(), true, false);
            b4.LinearLoadCasesList.Add(beamLoad);
            Model.InputFiniteElements.Add(b4);

            //Add loads to beams
            LinearLoadCaseForSuperFrameElement llcsfe = new LinearLoadCaseForSuperFrameElement("lc1", LoadCaseType.DEAD);
            llcsfe.UniformLoad.UniformLoadsZ.Add(new FrameSuperUniformLoad(0, 1, -10, -10, LoadDefinitionFromStartingNode.Relatively, LoadCordinateSystem.Global));
            b1a.LinearLoadCasesList.Add(llcsfe);
            b1b.LinearLoadCasesList.Add(llcsfe);
            b1c.LinearLoadCasesList.Add(llcsfe);
            b2.LinearLoadCasesList.Add(llcsfe);
            b3.LinearLoadCasesList.Add(llcsfe);
            b4.LinearLoadCasesList.Add(llcsfe);

            Model.Solve();

            //-------OBTAIN RESULTS-------
            double[] Min, Max;

            //Beam forces
            b1a.GetInternalForcesForLoadCase(0, "lc1", out Min, out Max, 1);

            //Foundation deflection at wall base x=3, y=1, z=0
            SuperNode nf = Model.InputNodes.Find(p => p.x == 3 && p.y == 1 && p.z == 0);
            nf.GetNodalDisplacementsForLoadCase("lc1", out Min, out Max, 1);

            //Estimation of soil stress at wall base
            double soilStress = -soilStiffnessPerArea * Min[2];


            //Get stresses at the base of wall (stresses of sheels are reported in the local coordinate system!!)
            ((ShellSuperElementTriangular)Model.InputFiniteElements.Find(p => ((ShellSuperElementTriangular)p).Node3.z == 1)).GetInternalStressesForLoadCase(0, 0, 0.125, CoordinateSystem.Local, 0, "lc1", out Min, out Max, 1);

            //Get forces at selected node (Fx,Fy,Fz,Mx,My,Mz are reported on the local or global coordinate system)
            ((ShellSuperElementTriangular)Model.InputFiniteElements.Find(p => ((ShellSuperElementTriangular)p).Node3.z == 1)).GetInternalForcesForLoadCase(ShellTriangularResultsLocation.Point1, CoordinateSystem.Global, "lc1", out Min, out Max, 1);
        }
        #endregion


        #region Form
        public Form1()
        {
            InitializeComponent();
        }
        private void lbl_Click(object sender, EventArgs e)
        {
            System.Diagnostics.Process.Start("https://www.engissol.com/3d-frame-library.html");
        }
        #endregion

    }
}