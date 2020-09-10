#GearFromCircle Python script by Thomas An 2020.09.08
#https://developer.rhino3d.com/guides/rhinopython/
#https://developer.rhino3d.com/api/RhinoScriptSyntax/
#Script calculations were informed by: Technical Drawing With Engineering Graphics 14edition (Giesecke, Mitchell, Spencer, Hill, Dygdon, Novak, Lockhart)

import rhinoscriptsyntax as rs, Rhino, math

#CLASS SECTION==================================================================
class TypeGear:
    def __init__(self):
        self.epsilon      = rs.UnitAbsoluteTolerance()
        self.curve        = None   #Holds the Rhino pitch curve object
        self.crvLen       = None
        self.isClosed     = None
        self.isCircle     = None
        self.PD           = 1.0    #Pitch diameter = given                      (propagate to -> MDL, BC, Tc)
        self.PA           = 20.0   #Pressure angle = 14.5, 20.0, 25.0           (propagate to -> minN, BC, DED)
        self.MDL          = 0.0588 #Module = PD / N                             (propagate to -> ADD, DED, OD, CP)
        self.N            = 17     #Number of teeth                             (propagate to -> MDL, Tc)
        self.maxN         = 1000
        self.minN         = 17     #Min number of teeth (depends on pressure angle) -> 2/sin^2(PA*pi/180)
        self.BC           = 0.9397 #Base circle = PD*cos(pi*PA/180). The bigger the pressure angle the further BC is to PD
        self.ADD          = 0.0588 #Addendum = MDL
        self.DED          = 0.0735 #Dedendum = 1.250*MDL use 1.157 for 14.5 PA  (propagate to -> RD)
        self.OD           = 1.1176 #Outside diameter = PD+2*MDL
        self.RD           = 0.8529 #Root diameter = PD-2*DED
        self.Tc           = 0.0922 #Chordal thickness = PD*sin(pi/(2*N))
        self.CP           = 0.1848 #Circular pitch = pi*MDL or crvLen/N          (propagate to -> MDL )
        self.CA           = 0.0    #Cone angle (0 to 90)
        self.origin       = None   #Pitch circle origin (world coordinates)
        self.normal       = None   #The normal of the plane the gear is on (this 3D point assumes world 0,0,0 as the first vector point)
        self.plane        = None   #The local plane where the gear is on. Origin of plane is set at center of gear.
        self.smpl         = 5      #Involute point samples (3 to 40)
        self.error        = False  #Setting this to true will cause exit without draw
        self.show         = {
          "PitchCir": False, 
          "BCcircle": False, 
          "ODcircle": False, 
          "RDcircle": False}
        self.involute     = {
          "startAngle" : None,
          "endAngle"   : None,
          "angleMod"   : None,
          "pointsLeft" : [],
          "pointsRight": [],
          "pointsTop"  : [],
          "ptsLeftDed" : [],
          "ptsRightDed": []}        #points are stored in world coordinates
    
    #Internal methods ----------------------------------------------------------
    
    #Compute the involute parameters
    def CalcInvolute(self):
        self.involute["startAngle"] = math.pi/2 + math.asin(self.Tc/self.PD) - self.PA*math.pi/180 + math.sqrt(math.pow(self.PD/self.BC,2) -1)
        self.involute["endAngle"]   = self.involute["startAngle"] - math.sqrt( math.pow(self.OD/self.BC,2) -1)
        self.involute["angleMod"]   = math.sqrt(math.pow(self.RD/self.BC,2) -1) if (self.RD > self.BC) else 0 

        loopStep = (self.involute["startAngle"] - self.involute["angleMod"] - self.involute["endAngle"])/self.smpl
        
        #Compute the left and right involute points and store them into their arrays (in world coordinates)
        for n in range(self.smpl+1):
          involuteHeight = math.sqrt( math.pow(self.BC/2 ,2)*(math.pow( n*loopStep + self.involute["angleMod"] ,2) + 1) )
          involuteHeightAngle = self.involute["startAngle"] - self.involute["angleMod"] - n*loopStep + math.atan( n*loopStep +  self.involute["angleMod"])
      
          samplePoint = self.CalcBevelPoint( [involuteHeight*math.cos(involuteHeightAngle), involuteHeight*math.sin(involuteHeightAngle), 0] )
          samplePtMir = [-samplePoint[0],samplePoint[1],samplePoint[2]] #Mirror about the Y-axis
          self.involute["pointsLeft"].append(rs.XformCPlaneToWorld(samplePoint, self.plane))
          self.involute["pointsRight"].append(rs.XformCPlaneToWorld(samplePtMir,self.plane))
        
        #Compute the top arc points and store them in their array (in world coordinates)
        midTopPoint = self.CalcBevelPoint([0,self.OD/2,0])
        self.involute["pointsTop"].append(self.involute["pointsLeft"][-1])
        self.involute["pointsTop"].append(self.involute["pointsRight"][-1])
        self.involute["pointsTop"].append(rs.XformCPlaneToWorld(midTopPoint,self.plane))
           
        #Compute the line segments for the dedendum
        if (self.RD < self.BC):
          samplePoint = self.CalcBevelPoint( [self.RD/2 * math.cos(self.involute["startAngle"]), self.RD/2 * math.sin(self.involute["startAngle"]), 0] )
          samplePtMir = [-samplePoint[0],samplePoint[1],samplePoint[2]] #Mirror about the Y-axis
          self.involute["ptsLeftDed"].append(self.involute["pointsLeft"][0])
          self.involute["ptsLeftDed"].append(rs.XformCPlaneToWorld(samplePoint,self.plane))
          self.involute["ptsRightDed"].append(self.involute["pointsRight"][0])
          self.involute["ptsRightDed"].append(rs.XformCPlaneToWorld(samplePtMir,self.plane))

    #Receives a 3Dpoint and tilts it by some angle
    def CalcBevelPoint (self, oldPoint):
        if (self.CA == 0): return oldPoint
        
        newPoint    = [0,0,0]
        delta       = [0,0]
        delta[1]    = math.sqrt( oldPoint[0]*oldPoint[0] + oldPoint[1]*oldPoint[1] ) - self.PD/2
        newPoint[2] = delta[1] * math.sin( self.CA * math.pi/180 )
        delta[0]    = delta[1] * math.cos( self.CA * math.pi/180 )
        newPoint[0] = (self.PD/2+delta[0]) / (self.PD/2+delta[1]) * oldPoint[0]
        newPoint[1] = (self.PD/2+delta[0]) / (self.PD/2+delta[1]) * oldPoint[1]
        return newPoint
        
    #Set methods ---------------------------------------------------------------
    def SetPlanarCurve(self, type="Any", guid=None):
        if (type=="Any"): prompt = "Select a planar pitch curve" 
        elif (type=="Circle"): prompt = "Select the pitch circle"
        
        newCurve = rs.GetCurveObject(prompt, True, True) if guid is None else [guid]
        if (newCurve is None): Rhino.RhinoApp.WriteLine("Exit: No curve was selected"); return False
        isPlanar = rs.IsCurvePlanar(newCurve[0])
        if (isPlanar == False): newCurve = None; Rhino.RhinoApp.WriteLine("Exit: Selected curve was not planar"); return False

        self.curve = newCurve #Accept the curve into the object because the curve exists and it is planar
        self.isClosed = rs.IsCurveClosed(self.curve[0])
        self.normal = rs.CurveNormal(self.curve[0]) #For non planar curves the script already exited.
        
        self.isCircle = rs.IsCircle(self.curve[0])
        if (type=="Circle" and self.isCircle != True): Rhino.RhinoApp.WriteLine("Exit: Selected curve was not a circle"); return False

        self.crvLen = rs.CurveLength(self.curve[0])
        if (self.isCircle == True): 
          Rhino.RhinoApp.WriteLine("Selected: Circle")
          self.origin=rs.CircleCenterPoint(self.curve[0])
          self.plane = rs.PlaneFromNormal(self.origin, self.normal, rs.CurveStartPoint(self.curve[0])-self.origin) #PlaneFromNormal(origin, normal, xaxis=None)
          self.SetPD() #Propagate the value updates
          self.SetBC() #Propagate the value updates
          return 
        if (self.isClosed == True): Rhino.RhinoApp.WriteLine("Selected: Closed non-circular planar curve"); return
        else: Rhino.RhinoApp.WriteLine("Selected: Open planar curve")
    
    #Set Pitch Diameter
    def SetPD(self, newPD=None):
        if (newPD is None and not self.isCircle): return #nothing to do
        if (newPD is None and self.isCircle): newPD = 2*rs.CircleRadius(self.curve[0]) #simple update/refresh of values
        if (newPD <= 0.0): Rhino.RhinoApp.WriteLine("Exit: Invalid Pitch Diameter"); return #nothing to do
        if (newPD == self.PD): return #Nothing to do
        
        self.PD = float(newPD)
        self.SetMDL(); self.SetBC(); self.SetTc()  #Propagate values refresh
    
    #Set Base circle
    def SetBC(self, newBC=None):
        if (newBC is None): self.BC = self.PD*math.cos(math.pi*self.PA/180); return #simple update/refresh of values
        if (newBC == self.BC): return #Nothing to do
    
    #Set Module
    def SetMDL(self, newMDL=None):
        if (newMDL is None): self.MDL = self.PD/self.N; self.SetADD(); self.SetDED(); self.SetOD(); self.SetCP(); return #simple update/refresh of values and propagate
        if (newMDL<=0): Rhino.RhinoApp.WriteLine("Module cannot be less than or 0.0"); return #Nothing to do
        if (newMDL==self.MDL): return #Nothing to do
        
        maxMDL  = self.PD/self.minN #so that gear teeth do not drop bellow min when preserving PitchCircle
        minMDL  = self.PD/self.maxN #so that gear teeth do not rise above max when preserving PitchCircle
        newN    = self.PD/newMDL    #the number of teeth needed with this new module
        newNint = int(round(newN,0))
        if (newMDL<minMDL or newMDL>maxMDL): self.SetPD(newMDL*self.N); return #Preserve the teeth by default
        
        #if the module value is within the min and max range then there is ambiguity about whether to preserve the PD or N in the expression
        #Ask the user
        Rhino.RhinoApp.WriteLine("[CurrentTeeth = "+str(self.N)+"]. [PitchCircleSize fits "+str(newNint)+" teeth]. Either option affects pitch diameter")
        strPromptMDL = "Prefer"
        strOption1   = "CurrentTeeth"    #Preserve original teeth number and keep the exact module
        strOption2   = "PitchCircleSize" #Preserve the new integer teeth number and keep the exact module
        strPrommptMDLoptions = [strOption1,strOption2]
        strPreservePick = rs.GetString(strPromptMDL,strPrommptMDLoptions[0],strPrommptMDLoptions)
        
        if(strPreservePick==strOption1): self.SetPD(newMDL*self.N); return #the newMDL value will be kept and stored after PD propagation
        if(strPreservePick==strOption2): self.SetN(newN); self.SetPD(newMDL*newNint); return #the MDL value will be kept and stored after PD propagation
        #if(strPreservePick=="PitchCircle"): self.SetN(newN); return #the newMDL value will be altered after N propagation
        
    def SetN(self, newN=None):
        if (newN is None or newN==self.N): return #nothing to do
        if (newN>self.maxN or newN<self.minN): Rhino.RhinoApp.WriteLine("Teeth number out of bounds: ("+str(self.minN)+" to "+str(self.maxN)+")"); return
        if (newN == self.N): return #Nothing to do
        self.N = int(round(newN,0))
        self.SetTc()
        self.SetMDL() #Propagate values refresh
     
    #Set Addendum
    def SetADD(self, newADD=None):
        if (newADD is None): self.ADD = self.MDL; return #simple update/refresh of values
        if (newADD == self.ADD): return #Nothing to do
    
    #Set Deddendum
    def SetDED(self, newDED=None):
        if (newDED is None): self.DED = 1.250*self.MDL if self.PA>14.5 else 1.157*self.MDL; self.SetRD(); return #simple update/refresh of values and propagate
        if (newDED == self.DED): return #Nothing to do
    
    #Set Root Diameter
    def SetRD(self, newRD=None):
        if (newRD is None): self.RD = self.PD-2*self.DED; return #simple update/refresh of values
        if (newRD == self.RD): return #Nothing to do
    
    #Set Outside Diameter
    def SetOD(self, newOD=None):
        if (newOD is None): self.OD = self.PD+2*self.MDL; return #simple update/refresh of values
        if (newOD == self.OD): return #Nothing to do
    
    #Set Circular Pitch
    def SetCP(self, newCP=None):
        if (newCP is None): self.CP = math.pi*self.MDL; return #simple update/refresh of values
        if (newCP<=0): Rhino.RhinoApp.WriteLine("Circular Pitch cannot be less than or 0.0"); return #Nothing to do
        if (newCP==self.CP): return #Nothing to do
        
        self.SetMDL(newCP/math.pi) #Actual CP value will change inside the MDL method
    
    #Set Chordal Thickness
    def SetTc(self, newTc=None):
        if (newTc is None): self.Tc = self.PD*math.sin(math.pi/(2*self.N)); return #simple update/refresh of values
        if (newTc == self.Tc): return #Nothing to do
    
    #Set Cone angle
    def SetCA(self, newCA=None):
        if (newCA is None or newCA == self.CA): return #Nothing to do
        if (newCA<0 or newCA>90): Rhino.RhinoApp.WriteLine("Gear teeth cone-angle out of range (0-90deg)"); return
        self.CA = newCA
    
    #Set Tooth involute curve point count
    def Setsmpl(self, newsmpl=None):
        if (newsmpl is None or newsmpl == self.smpl): return #Nothing to do
        if (newsmpl<3 or newsmpl>40): Rhino.RhinoApp.WriteLine("Involute curve point count out of range (range: 3-40samples)"); return
        self.smpl = newsmpl
    
    #Set Pressure angle
    def SetPA(self, newPA=None):
        PAchoices = (14.5,20.0,25.0)
        if (newPA is None or newPA==self.PA): return #Nothing to do (PA is not computed from anything else)
        if (newPA == self.PA): return #Nothing to do
        if (not newPA in PAchoices): Rhino.RhinoApp.WriteLine("Pressure angle was not one of these"+str(PAchoices)); return
        self.PA = newPA
        self.SetminN(); self.SetBC(); self.SetDED() #Propagate changes
        
    #Set Min number of teeth
    def SetminN(self, newminN=None):
        absoluteMin = int(2/(math.sin(self.PA*math.pi/180)**2));
        if (newminN is None): self.minN = absoluteMin; return #simple update/refresh of values
        if (newMin == self.minN): return #Nothing to do
        if (newMin < absoluteMin or newMin>maxN-1): Rhino.RhinoApp.WriteLine("Chosen minimum teeth number is out of range ("+str(absoluteMin)+", "+str(self.maxN)+")"); return
        self.minN = newMin

    def SetError(self):
        self.error = True
    
    #Get methods
    def GetSummary(self):
        infoStr  = 'Summary: '
        infoStr += '[Pitch Diam='+str(round(self.PD,3))+'] '
        infoStr += '[Teeth='+str(int(self.N))+'] '
        infoStr += '[Module='+str(round(self.MDL,3))+'] '
        infoStr += '[CircPitch='+str(round(self.CP,3))+', ToothThickness='+str(round(self.Tc,3))+'] '
        infoStr += '[Pressure angle='+str(round(self.PA,1))+'] '
        infoStr += '[Bevel angle='+str(round(self.CA,1))+'] '
        infoStr += '[Samples='+str(int(self.smpl))+']'
        
        Rhino.RhinoApp.WriteLine(infoStr)
    
    def GetFullInfo(self):
        infoStr  = 'Curve is closed = '+str(self.isClosed)+'\n'
        infoStr += 'Curve is circle = '+str(self.isCircle)+'\n'
        infoStr += 'Pitch diameter = '+str(self.PD)+'\n'
        infoStr += 'Pressure angle = '+str(self.PA)+'\n'
        infoStr += 'Module = '+str(self.MDL)+'\n'
        infoStr += 'Number of teeth = '+str(self.N)+'\n'
        infoStr += 'Max number of teeth = '+str(self.maxN)+'\n'
        infoStr += 'Min number of teeth = '+str(self.minN)+'\n'
        infoStr += 'Base circle diameter = '+str(self.BC)+'\n'
        infoStr += 'Addendum = '+str(self.ADD)+'\n'
        infoStr += 'Dedendum = '+str(self.DED)+'\n'
        infoStr += 'Outside diamter = '+str(self.OD)+'\n'
        infoStr += 'Root diameter = '+str(self.RD)+'\n'
        infoStr += 'Chordal thickness = '+str(self.Tc)+'\n'
        infoStr += 'Circular pitch = '+str(self.CP)+'\n'
        infoStr += 'Cone angle = '+str(self.CA)+'\n'
        
        rs.MessageBox(infoStr)
    
    #Draw the gear on the Rhino viewport
    def Draw(self):
      if (self.error == True): return #Nothing to do
      self.CalcInvolute()
      
      rs.EnableRedraw(False)
      #Show various gear related circles on the viewport
      if (self.show["PitchCir"]==True or math.fabs(self.PD/2 - rs.CircleRadius(self.curve[0])) > self.epsilon): rs.SelectObject(rs.AddCircle(self.plane,self.PD/2)) #if the pitch circle has changed --> show it
      if (self.show["BCcircle"]==True): rs.SelectObject(rs.AddCircle(self.plane,self.BC/2))
      if (self.show["ODcircle"]==True): rs.SelectObject(rs.AddCircle(self.plane,self.OD/2))
      if (self.show["RDcircle"]==True): rs.SelectObject(rs.AddCircle(self.plane,self.RD/2))
      
      #Make the first gear tooth
      #Note: (gearTooth is an array of curve IDs [left involute, right involute, top arc, leftDedendumLine, RightDedendumLine])
      gearTooth = [rs.AddInterpCurve(self.involute["pointsLeft"],3,1), rs.AddInterpCurve(self.involute["pointsRight"],3,1)]
      gearTooth.append(rs.AddArc3Pt(self.involute["pointsTop"][0], self.involute["pointsTop"][1], self.involute["pointsTop"][2]))
      if (len(self.involute["ptsLeftDed"])>0): gearTooth.append(rs.AddLine(self.involute["ptsLeftDed"][0], self.involute["ptsLeftDed"][1]) )
      if (len(self.involute["ptsRightDed"])>0): gearTooth.append(rs.AddLine(self.involute["ptsRightDed"][0], self.involute["ptsRightDed"][1]) )
      
      #Join all curves of the first tooth and add it in the wholeGear array
      wholeGear = [rs.JoinCurves(gearTooth, True)[0],[]] #contains the first tooth and space for its right root arc
      
      #Create a datum point (in local coordinates) on the root circle and to the right of the first gear tooth
      #This becomes the seed for all midPoints of the root arcs
      datumPoint = rs.VectorRotate([0, self.RD/2, 0], -180/self.N, [0,0,1]) #VectorRotate works in degrees
      
      #Loop through each gear tooth
      for n in range(1, self.N+1):
        
        #Compute the midpoint of the root arc
        midPoint = rs.VectorRotate(datumPoint, n*360/self.N, [0,0,1]) #local coordinates
        midPoint = rs.XformCPlaneToWorld(self.CalcBevelPoint(midPoint), self.plane) #world coordinates
        
        if (n < self.N):
          wholeGear.append( rs.RotateObject(wholeGear[0], self.plane[0], n*360/self.N, self.plane[3], True) ) #Rotate and copy (in world coordinates) 
          wholeGear.append( rs.AddArc3Pt(rs.CurveEndPoint(wholeGear[-1]), rs.CurveStartPoint(wholeGear[-3]), midPoint) )  #AddArc3Pt(start, end, point_on_arc)
        else:
          wholeGear[1] = ( rs.AddArc3Pt(rs.CurveEndPoint(wholeGear[0]), rs.CurveStartPoint(wholeGear[-2]), midPoint) )  
      
      #Join the all the gear curves together
      wholeGear = rs.JoinCurves(wholeGear, True)[0]
      rs.UnselectObject(self.curve[0])
      rs.SelectObject(wholeGear)
      rs.EnableRedraw(True)

#FUNCTIONS SECTION==============================================================
#User input---------------------------------------------------------------------
def UserInput(gear):

    #Get a circle curve from the user
    if (gear.SetPlanarCurve("Circle")==False): gear.SetError(); return

    strMainMenu = "Main menu"
    strMainOptions = ["Teeth", "Module", "Pitch", "PressureAngle", "BevelAngle", "Accuracy"]
    
    while True:
      gear.GetSummary();
      strMainMenuPick = rs.GetString(strMainMenu,None,strMainOptions)

      if (strMainMenuPick == "Teeth"): 
        strPrompt = "Number of teeth ("+str(gear.minN)+" to "+str(gear.maxN)+")"
        teethCount = rs.GetInteger(strPrompt, gear.N, gear.minN, gear.maxN)
        gear.SetN(teethCount)
        continue
      if (strMainMenuPick == "Module"):
        maxMDL = gear.PD/gear.minN #so that gear teeth do not drop bellow min
        minMDL = gear.PD/gear.maxN #so that gear teeth do not rise above max
        Rhino.RhinoApp.WriteLine("Module must be within range ["+str(round(minMDL,4))+", "+str(round(maxMDL,4))+"] otherwise the PitchCircle will be affected")
        fltPrompt = "Gear Module (PitchDia/Teeth)"
        module = rs.GetReal(fltPrompt,gear.MDL)
        gear.SetMDL(module)
        continue
      if (strMainMenuPick == "Pitch"):
        maxCP = math.pi*gear.PD/gear.minN #so that gear teeth do not drop bellow min
        minCP = math.pi*gear.PD/gear.maxN #so that gear teeth do not rise above max
        Rhino.RhinoApp.WriteLine("The current pitch-circle can support pitch in the range ["+str(round(minCP,4))+", "+str(round(maxCP,4))+"] otherwise the PitchCircle will be affected")
        fltPrompt = "Circular Pitch (tooth to tooth)"
        pitch = rs.GetReal(fltPrompt,gear.CP)
        gear.SetCP(pitch)
        continue
      if (strMainMenuPick == "PressureAngle"):
        strPAprompt = "Pressure angle choices (low = 14.5, mid = 20.0, high = 25.0)"
        strPAoptions = ["low","mid","high"]
        PAchoice = rs.GetString (strPAprompt,strPAoptions[1],strPAoptions) #Rhino.GetOption(strPAprompt,strPAoptions[1],strPAoptions)
        if (PAchoice=="low"): PAchoice=14.5
        elif (PAchoice=="mid"): PAchoice=20.0
        elif (PAchoice=="high"): PAchoice=25.0
        gear.SetPA(PAchoice)
        continue
      if (strMainMenuPick == "BevelAngle"):
        strPrompt = "Gear teeth cone angle (0"+u'\xb0'+" - 90"+ u'\xb0'+")"
        bevelChoice = rs.GetInteger(strPrompt, gear.CA, 0, 90)
        gear.SetCA(bevelChoice)
        continue
      if (strMainMenuPick == "Accuracy"):
        strPrompt = "Tooth involute curve point count (3-40samples)"
        curveSamples = rs.GetInteger(strPrompt, gear.smpl, 3, 40)
        gear.Setsmpl(curveSamples)
        continue
      if (strMainMenuPick == ""):
        break
      if (strMainMenuPick is None):
        Rhino.RhinoApp.WriteLine ("Script Aborted")
        gear.error = True;
        break

#MAIN function------------------------------------------------------------------
def GearFromCircle():
    gear = TypeGear() #Gear object
    UserInput(gear)   #Collect user choices for the gear
    gear.Draw()       #Draw the involute gear on the Rhino viewport
    
    #Wrap up
    if (gear.error == False):
      Rhino.RhinoApp.WriteLine ("Script completed successfully")
      gear.GetSummary()

#RUN program
GearFromCircle()