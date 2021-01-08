# Racing game met ML-Agent

## Team JPJB

### Inhoudstafel

- [Groepsleden](0.-Groepsleden)
- [Inleiding](1.-Inleiding)
- [Methoden](2.-Methoden)
  - [Installatie](2.1-Installatie)
  - [Verloop simulatie](2.2-VerloopSimulatie)
  - [Observaties, acties, beloningen](2.3-ObservatiesActiesBeloningen)
  - [Beschrijving objecten](2.4-BeschrijvingObjecten)
  - [Scripts](2.5-Scripts)
  - [Afwijking one-pager](2.6-AfwijkingOnePager)
- [Trainen](3.-Trainen)
- [Resultaten](4.-Resultaten)
- [Conclusie](5.-Conclusie)
- [Bronvermelding](6.-Bronvermelding)

### 0. Groepsleden

S106673 - Jana Van Goethem
S106707 - Pieter Ongenae
S106999 - Brent Janssens
S107615 - Vincent Hertens
S108900 - Jonas Van Haute

### 1 Inleiding

Het doel van het spel is een speler te laten racen tegen een ML-Agent in een virtuele wereld door middel van een VR-bril. Men moet de vooropgestelde circuits afleggen en de finish bereiken. Om de Agent te leren het circuit succesvol af te leggen gaan we werken met een beloningssysteem en checkpoints. Bij het overgaan van het checkpoint krijgt de Agent punten, bij het aanraken van muren verliest de Agent punten. Het doel is om als eerste het circuit af te leggen.

### 2.Samenvatting

In deze tutorial gaat u leren een racing game met ML-Agent te maken in Unity. In deze game zal u kunnen racen met een tegenstander bediend door de computer die getraind worden a.d.h.v. Machine Learning.

> OPMERKING! Er wordt in deze tutorial ervan uit gegaan dat u voorkennis heeft van de Unity omgeving, scripting (C#) en ML-Agents.

### 2.Methoden

#### 2.1 Installatie

| Tools                  | Versie    | Link                                                                    |
| ---------------------- | --------- | ----------------------------------------------------------------------- |
| Unity                  | 2019.4.11 | <https://unity.com/>                                                    |
| Visual Studio          | 2019      | <https://visualstudio.microsoft.com/>                                   |
| Python                 | 3.8.5     | <https://www.python.org/>                                               |
| ML Agents              | 1.0.5     | <https://github.com/Unity-Technologies/ml-agents>                       |
| XR Interaction Toolkit | 0.9.4     | <https://docs.unity3d.com/Manual/com.unity.xr.interaction.toolkit.html> |
| SideQuest              | 0.10.18   | <https://sidequestvr.com/>                                              |

De objecten/scripts die gebruikt worden in dit project zijn gebundeld in een custom asset package, het is aangeraden om deze te installeren.
Dit kan u doen door te navigeren naar Assests > Import Package > Custom package.
De custom asset package kan u vinden onder de naam RacingGame.unitypackage.

#### 2.2 Verloop simulatie

Tijdens het starten van een simulatie zal je op circuit 1 van de 3 terechtkomen en kunnen racen tegen een AI. Als de speler of de AI de finish bereikt gaat de speler automatisch naar het volgende circuit. Nadat alle 3 de circuits doorlopen zijn is het spel gedaan.

#### 2.3 Observaties, acties, beloningen

##### Observaties

De ML Agent kan maar 1 object observeren, namelijk de muren van de weg. Dit dankzij de Ray Perception Sensor 3D.

> Instellingen van de Ray Perception Sensor 3D zijn te vinden onder de titel 'beschrijving objecten'.

##### Acties

Zowel de player als de ML Agent hebben keuze uit dezelfde acties.

| Omschrijving Actie | Shortcut (Oculus Quest Controllers)          |
| ------------------ | -------------------------------------------- |
| Vooruit rijden     | Rechter Trigger                              |
| Remmen             | Linker Trigger                               |
| Achteruitrijden    | Linker Trigger ingedrukt houden na stilstaan |
| Draaien (Links)    | Rechter Joystick naar links bewegen          |
| Draaien (Rechts)   | Rechter Joystick naar rechts bewegen         |

##### Beloningen

Voor het trainen van de Agent wordt het volgende beloningssyteem gebruikt:

| Omschrijving     | Beloning |
| ---------------- | -------- |
| contact met muur | -2f      |
| achteruitrijden  | -0.001f  |
| stilstaan        | -0.1f    |
| checkpoint       | +1f      |
| Finish           | +2f      |
| vooruitrijden    | +0.0001f |

#### 2.4 Beschrijving objecten

Zoals reeds vermeld zijn alle objecten die gebruikt zijn in dit project gebundeld in een custom asset package. Op deze manier moet je niet alle objecten zelf gaan maken maar kan je gebruik maken van de prefabs. De scripts waarvan de objecten gebruik maken kan u vinden onder de titel 'Scripts'.

##### Player Car

![PlayerCar](./images/PlayerCar.png)
![PlayerCarSettings](./images/PlayerCarSettings.png)
![PlayerCarWheelColliders](./images/PlayerCarWheelColliders.png)

De player car beschikt over een Player Controller script dat ervoor zorgt dat de speler de auto kan bedienen aan de hand van de Oculus controllers. De player car beschikt ook over zowel een Rigidbody als een Box Collider. De player car wordt aangedreven door 4 wielen. Deze wielen kan u terugvinden in de prefabs. De colliders van deze wielen dienen toegekend te worden aan de Player Controller onder 'Wheel colliders'

##### AI Car

![AICar](./images/AICar.png)
![AICarSettings1](./images/AICarSettings1.png)
![AICarSettings2](./images/AICarSettings2.png)

De AI Car beschikt over verschillende scripts namelijk een Car controller, Ray Perception Sensor 3D, Behavior Parameters, Agent en Decision Requester. De Car controller is hier een variant op de Player Controller. Deze is aangepast zodat de AI car kan worden getrained. De AI car wordt aangedreven door 4 wielen. Deze wielen kan u terugvinden in de prefabs. De colliders van deze wielen dienen toegekend te worden aan de Car controller onder 'Wheel colliders'.

##### Weg

![Weg](./images/Weg.png)
![WegSettings](./images/WegSettings.png)

U kan een circuit samenstellen door een combinatie te gebruiken van de verschillende onderdelen (rechte weg, bochten, etc).
Vergeet niet om de weg de tag van "Wall" te geven. Deze tag wordt in de scripts gebruikt om de Agent af te straffen bij contact.

##### Finish

![Finish](./images/Finish.png)

In de Finish prefab zit een plane, belangrijk is dat deze de tag FinishLine heeft. Deze tag wordt gebruikt in de scripts om ervoor te zorgen dat als een auto over de finish rijdt het volgende circuit wordt ingeladen.

##### Checkpoint

![Checkpoint](./images/Checkpoint.png)

Om de checkpoint doorzichtig te maken kan je de mesh renderer afzetten.
De checkpoints zijn dus volledig onzichtbaar en worden enkel gebruikt voor het trainen van de Agent en om ervoor te zorgen dat de Agent of de speler niet achteruit over de finishlijn kunnen rijden.

##### Camera

![Camera](./images/Camera.png)
![CameraSettings](./images/CameraSettings.png)

Deze camera is geen gewone camera maar een XR Rig camera, deze zal ervoor zorgen dat in de VR bril een speler rond kan zien. De camera bevat het Camera Follow script, dit zorgt ervoor dat in de VR bril de camera de auto volgt wanneer deze beweegt.

##### Omgeving

![Bomen](./images/Bomen.png)

De omgevingen bevatten verschillende soorten bomen en bergen (Terrain) die ook in de assets kunnen worden teruggevonden. Deze zijn niet noodzakelijk om te gebruiken.

#### 2.5 Scripts

##### Player Controller

> REFERENTIE Dit script bevat grotendeels code van de CarController in de 'Unity Standard Assets' asset. Deze is aangepast om te laten werken met de VR-bril. Zie bronvermelding.
> REFERENTIE Dit script bevat code van de XR Input manual. Zie bronvermelding.

- Speler kan kiezen tussen 3 soorten aandrijving, voorwielaandrijving, achterwielaandrijving of vierwielaandrijving.

```c#
internal enum CarDriveType
{
    FrontWheelDrive,
    RearWheelDrive,
    FourWheelDrive
}
```

- Speler kan kiezen tussen 2 snelheidstypes, kilometer per uur of mijl per uur.

```c#
internal enum SpeedType
{
    MPH,
    KPH
}
```

- Benodigde properties.

```c#
public class PlayerController : MonoBehaviour
{
    [SerializeField] private CarDriveType m_CarDriveType = CarDriveType.FourWheelDrive;
    [SerializeField] private WheelCollider[] m_WheelColliders = new WheelCollider[4];
    [SerializeField] private GameObject[] m_WheelMeshes = new GameObject[4];
    [SerializeField] private WheelEffects[] m_WheelEffects = new WheelEffects[4];
    [SerializeField] private Vector3 m_CentreOfMassOffset;
    [SerializeField] private float m_MaximumSteerAngle;
    [Range(0, 1)] [SerializeField] private float m_SteerHelper; // 0 is raw physics , 1 the car will grip in the direction it is facing
    [Range(0, 1)] [SerializeField] private float m_TractionControl; // 0 is no traction control, 1 is full interference
    [SerializeField] private float m_FullTorqueOverAllWheels;
    [SerializeField] private float m_ReverseTorque;
    [SerializeField] private float m_MaxHandbrakeTorque;
    [SerializeField] private float m_Downforce = 100f;
    [SerializeField] private SpeedType m_SpeedType;
    [SerializeField] private float m_Topspeed = 200;
    [SerializeField] private static int NoOfGears = 5;
    [SerializeField] private float m_RevRangeBoundary = 1f;
    [SerializeField] private float m_SlipLimit;
    [SerializeField] private float m_BrakeTorque;

    private bool checkpoint1 = false;
    private bool checkpoint2 = false;
    private bool checkpoint3 = false;
    private bool checkpoint4 = false;
    private InputDevice rightController;
    private InputDevice leftController;

    private Quaternion[] m_WheelMeshLocalRotations;
    private Vector3 m_Prevpos, m_Pos;
    private float m_SteerAngle;
    private int m_GearNum;
    private float m_GearFactor;
    private float m_OldRotation;
    private float m_CurrentTorque;
    private Rigidbody m_Rigidbody;
    private const float k_ReversingThreshold = 0.01f;

    public bool Skidding { get; private set; }
    public float BrakeInput { get; private set; }
    public float CurrentSteerAngle { get { return m_SteerAngle; } }
    public float CurrentSpeed { get { return m_Rigidbody.velocity.magnitude * 2.23693629f; } }
    public float MaxSpeed { get { return m_Topspeed; } }
    public float Revs { get; private set; }
    public float AccelInput { get; private set; }
```

- Declaratie controllers + toekenning.

```c#
void Start()
{
    List<InputDevice> righthandControllers = new List<InputDevice>();
    List<InputDevice> lefthandControllers = new List<InputDevice>();
    InputDevices.GetDevices(righthandControllers);
    InputDevices.GetDevices(lefthandControllers);

    InputDeviceCharacteristics rightControllerCharacteristics = InputDeviceCharacteristics.Right;
    InputDevices.GetDevicesWithCharacteristics(rightControllerCharacteristics, righthandControllers);

    InputDeviceCharacteristics leftControllerCharacteristics = InputDeviceCharacteristics.Left;
    InputDevices.GetDevicesWithCharacteristics(leftControllerCharacteristics, lefthandControllers);

    if (righthandControllers.Count > 0)
    {
        rightController = righthandControllers[0];
    }

    if (lefthandControllers.Count > 0)
    {
        leftController = lefthandControllers[0];
    }

    m_WheelMeshLocalRotations = new Quaternion[4];
    for (int i = 0; i < 4; i++)
    {
        m_WheelMeshLocalRotations[i] = m_WheelMeshes[i].transform.localRotation;
    }
    m_WheelColliders[0].attachedRigidbody.centerOfMass = m_CentreOfMassOffset;

    m_MaxHandbrakeTorque = float.MaxValue;

    m_Rigidbody = GetComponent<Rigidbody>();
    m_CurrentTorque = m_FullTorqueOverAllWheels - (m_TractionControl * m_FullTorqueOverAllWheels);
}
```

- Versnellingsbak van de auto.

```c#
private void GearChanging()
{
    float f = Mathf.Abs(CurrentSpeed / MaxSpeed);
    float upgearlimit = (1 / (float)NoOfGears) * (m_GearNum + 1);
    float downgearlimit = (1 / (float)NoOfGears) * m_GearNum;

    if (m_GearNum > 0 && f < downgearlimit)
    {
        m_GearNum--;
    }

    if (f > upgearlimit && (m_GearNum < (NoOfGears - 1)))
    {
        m_GearNum++;
    }
}
```

```c#
// simple function to add a curved bias towards 1 for a value in the 0-1 range
private static float CurveFactor(float factor)
{
    return 1 - (1 - factor) * (1 - factor);
}
```

```c#
// unclamped version of Lerp, to allow value to exceed the from-to range
private static float ULerp(float from, float to, float value)
{
    return (1.0f - value) * from + value * to;
}
```

```c#
private void CalculateGearFactor()
{
    float f = (1 / (float)NoOfGears);
    // gear factor is a normalised representation of the current speed within the current gear's range of speeds.
    // We smooth towards the 'target' gear factor, so that revs don't instantly snap up or down when changing gear.
    var targetGearFactor = Mathf.InverseLerp(f * m_GearNum, f * (m_GearNum + 1), Mathf.Abs(CurrentSpeed / MaxSpeed));
    m_GearFactor = Mathf.Lerp(m_GearFactor, targetGearFactor, Time.deltaTime * 5f);
}
```

```c#
private void CalculateRevs()
{
    // calculate engine revs (for display / sound)
    // (this is done in retrospect - revs are not used in force/power calculations)
    CalculateGearFactor();
    var gearNumFactor = m_GearNum / (float)NoOfGears;
    var revsRangeMin = ULerp(0f, m_RevRangeBoundary, CurveFactor(gearNumFactor));
    var revsRangeMax = ULerp(m_RevRangeBoundary, 1f, gearNumFactor);
    Revs = ULerp(revsRangeMin, revsRangeMax, m_GearFactor);
}
```

- Dit deel zorgt ervoor dat de auto zich zal bewegen aan de hand van input.

```c#
public void Move(float steering, float accel, float footbrake, float handbrake)
{
    for (int i = 0; i < 4; i++)
    {
        Quaternion quat;
        Vector3 position;
        m_WheelColliders[i].GetWorldPose(out position, out quat);
        m_WheelMeshes[i].transform.position = position;
        m_WheelMeshes[i].transform.rotation = quat;
    }

    //clamp input values
    steering = Mathf.Clamp(steering, -1, 1);
    AccelInput = accel = Mathf.Clamp(accel, 0, 1);
    BrakeInput = footbrake = -1 * Mathf.Clamp(footbrake, -1, 0);
    handbrake = Mathf.Clamp(handbrake, 0, 1);

    //Set the steer on the front wheels.
    //Assuming that wheels 0 and 1 are the front wheels.
    m_SteerAngle = steering * m_MaximumSteerAngle;
    m_WheelColliders[0].steerAngle = m_SteerAngle;
    m_WheelColliders[1].steerAngle = m_SteerAngle;

    SteerHelper();
    ApplyDrive(accel, footbrake);
    CapSpeed();

    //Set the handbrake.
    //Assuming that wheels 2 and 3 are the rear wheels.
    if (handbrake > 0f)
    {
        var hbTorque = handbrake * m_MaxHandbrakeTorque;
        m_WheelColliders[2].brakeTorque = hbTorque;
        m_WheelColliders[3].brakeTorque = hbTorque;
    }


    CalculateRevs();
    GearChanging();

    AddDownForce();
    CheckForWheelSpin();
    TractionControl();
}
```

- Maximale snelheid auto.

```c#
private void CapSpeed()
{
    float speed = m_Rigidbody.velocity.magnitude;
    switch (m_SpeedType)
    {
        case SpeedType.MPH:

            speed *= 2.23693629f;
            if (speed > m_Topspeed)
                m_Rigidbody.velocity = (m_Topspeed / 2.23693629f) * m_Rigidbody.velocity.normalized;
            break;

        case SpeedType.KPH:
            speed *= 3.6f;
            if (speed > m_Topspeed)
                m_Rigidbody.velocity = (m_Topspeed / 3.6f) * m_Rigidbody.velocity.normalized;
            break;
    }
}
```

- Acceleratie van de auto.

```c#
private void ApplyDrive(float accel, float footbrake)
{

    float thrustTorque;
    switch (m_CarDriveType)
    {
        case CarDriveType.FourWheelDrive:
            thrustTorque = accel * (m_CurrentTorque / 4f);
            for (int i = 0; i < 4; i++)
            {
                m_WheelColliders[i].motorTorque = thrustTorque;
            }
            break;

        case CarDriveType.FrontWheelDrive:
            thrustTorque = accel * (m_CurrentTorque / 2f);
            m_WheelColliders[0].motorTorque = m_WheelColliders[1].motorTorque = thrustTorque;
            break;

        case CarDriveType.RearWheelDrive:
            thrustTorque = accel * (m_CurrentTorque / 2f);
            m_WheelColliders[2].motorTorque = m_WheelColliders[3].motorTorque = thrustTorque;
            break;

    }

    for (int i = 0; i < 4; i++)
    {
        if (CurrentSpeed > 5 && Vector3.Angle(transform.forward, m_Rigidbody.velocity) < 50f)
        {
            m_WheelColliders[i].brakeTorque = m_BrakeTorque * footbrake;
        }
        else if (footbrake > 0)
        {
            m_WheelColliders[i].brakeTorque = 0f;
            m_WheelColliders[i].motorTorque = -m_ReverseTorque * footbrake;
        }
    }
}
```

- Stuur helper.

```c#
private void SteerHelper()
{
    for (int i = 0; i < 4; i++)
    {
        WheelHit wheelhit;
        m_WheelColliders[i].GetGroundHit(out wheelhit);
        if (wheelhit.normal == Vector3.zero)
            return; // wheels arent on the ground so dont realign the rigidbody velocity
    }

    // this if is needed to avoid gimbal lock problems that will make the car suddenly shift direction
    if (Mathf.Abs(m_OldRotation - transform.eulerAngles.y) < 10f)
    {
        var turnadjust = (transform.eulerAngles.y - m_OldRotation) * m_SteerHelper;
        Quaternion velRotation = Quaternion.AngleAxis(turnadjust, Vector3.up);
        m_Rigidbody.velocity = velRotation * m_Rigidbody.velocity;
    }
    m_OldRotation = transform.eulerAngles.y;
}
```

- Dit zorgt ervoor dat de auto onder snelheid harder op de weg wordt gedrukt om grip te maximaliseren.

```c#
// this is used to add more grip in relation to speed
private void AddDownForce()
{
    m_WheelColliders[0].attachedRigidbody.AddForce(-transform.up * m_Downforce *
                                                    m_WheelColliders[0].attachedRigidbody.velocity.magnitude);
}
```

- Wielspin wanneer een auto met volle snelheid vertrekt of hard remt.

```c#
// checks if the wheels are spinning and is so does three things
// 1) emits particles
// 2) plays tiure skidding sounds
// 3) leaves skidmarks on the ground
// these effects are controlled through the WheelEffects class
private void CheckForWheelSpin()
{
    // loop through all wheels
    for (int i = 0; i < 4; i++)
    {
        WheelHit wheelHit;
        m_WheelColliders[i].GetGroundHit(out wheelHit);

        // is the tire slipping above the given threshhold
        if (Mathf.Abs(wheelHit.forwardSlip) >= m_SlipLimit || Mathf.Abs(wheelHit.sidewaysSlip) >= m_SlipLimit)
        {
            m_WheelEffects[i].EmitTyreSmoke();

            // avoiding all four tires screeching at the same time
            // if they do it can lead to some strange audio artefacts
            if (!AnySkidSoundPlaying())
            {
                m_WheelEffects[i].PlayAudio();
            }
            continue;
        }

        // if it wasnt slipping stop all the audio
        if (m_WheelEffects[i].PlayingAudio)
        {
            m_WheelEffects[i].StopAudio();
        }
        // end the trail generation
        m_WheelEffects[i].EndSkidTrail();
    }
}
```

- Tractiecontrole om te vermijden dat de auto grip verliest.

```c#
// crude traction control that reduces the power to wheel if the car is wheel spinning too much
private void TractionControl()
{
    WheelHit wheelHit;
    switch (m_CarDriveType)
    {
        case CarDriveType.FourWheelDrive:
            // loop through all wheels
            for (int i = 0; i < 4; i++)
            {
                m_WheelColliders[i].GetGroundHit(out wheelHit);

                AdjustTorque(wheelHit.forwardSlip);
            }
            break;

        case CarDriveType.RearWheelDrive:
            m_WheelColliders[2].GetGroundHit(out wheelHit);
            AdjustTorque(wheelHit.forwardSlip);

            m_WheelColliders[3].GetGroundHit(out wheelHit);
            AdjustTorque(wheelHit.forwardSlip);
            break;

        case CarDriveType.FrontWheelDrive:
            m_WheelColliders[0].GetGroundHit(out wheelHit);
            AdjustTorque(wheelHit.forwardSlip);

            m_WheelColliders[1].GetGroundHit(out wheelHit);
            AdjustTorque(wheelHit.forwardSlip);
            break;
    }
}
```

- Methode die gebruikt wordt door de tractiecontrole methode om het accelereren te stoppen zodat de auto terug grip krijgt.

```c#
private void AdjustTorque(float forwardSlip)
{
    if (forwardSlip >= m_SlipLimit && m_CurrentTorque >= 0)
    {
        m_CurrentTorque -= 10 * m_TractionControl;
    }
    else
    {
        m_CurrentTorque += 10 * m_TractionControl;
        if (m_CurrentTorque > m_FullTorqueOverAllWheels)
        {
            m_CurrentTorque = m_FullTorqueOverAllWheels;
        }
    }
}
```

- Slip geluid wanneer de banden geen tractie meer hebben.

```c#
private bool AnySkidSoundPlaying()
{
    for (int i = 0; i < 4; i++)
    {
        if (m_WheelEffects[i].PlayingAudio)
        {
            return true;
        }
    }
    return false;
}
```

```c#
private void FixedUpdate()
{
    MovePlayer();
}
```

- Input van de Oculus controller, deze wordt doorgegeven aan de Move die de beweging in gang zet.

```c#
private void MovePlayer()
{
    rightController.TryGetFeatureValue(CommonUsages.trigger, out float forwardValue);
    leftController.TryGetFeatureValue(CommonUsages.trigger, out float backwardValue);
    rightController.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 horizontalValue);
    rightController.TryGetFeatureValue(CommonUsages.primaryButton, out bool handbrakeValue);

    float v = forwardValue - backwardValue;
    float h = horizontalValue.x;
    float handbrake = handbrakeValue ? 1f : 0f;

    //float h = CrossPlatformInputManager.GetAxis("Horizontal");
    //float v = CrossPlatformInputManager.GetAxis("Vertical");
    //float handbrake = CrossPlatformInputManager.GetAxis("Jump");

    Move(h, v, v, handbrake);
}
```

- Op elk circuit zijn 4 vaste checkpoints ingebouwd, wanneer een auto hier voorbij rijdt veranderd de status hiervan. Pas als alle 4 de checkpoints zijn gepasseerd kan over de finish worden gereden en wordt het volgende circuit geladen.

```c#
private void OnTriggerEnter(Collider other)
{
    if (other.gameObject.CompareTag("Checkpoint1"))
    {
        checkpoint1 = true;
    }

    if (other.gameObject.CompareTag("Checkpoint2"))
    {
        checkpoint2 = true;
    }

    if (other.gameObject.CompareTag("Checkpoint3"))
    {
        checkpoint3 = true;
    }

    if (other.gameObject.CompareTag("Checkpoint4"))
    {
        checkpoint4 = true;
    }
    if (other.gameObject.CompareTag("FinishLine") && checkpoint1 == true && checkpoint2 == true && checkpoint3 == true && checkpoint4 == true)
    {
        UnityEngine.SceneManagement.SceneManager.LoadScene(UnityEngine.SceneManagement.SceneManager.GetActiveScene().buildIndex + 1);
    }
}
```

##### Car Controller (ML Agents controller)

> REFERENTIE Dit script bevat grotendeels code van de CarController in de 'Unity Standard Assets' asset. Zie bronvermelding

Dit script is hetzelfde als de Player Controller script, enkel onderstaande code is anders voor dit script.

- Rewardsysteem voor de Agent. Waarbij deze een beloning krijgt of afgestraft wordt voor bepaalde acties.

```c#
public override void OnActionReceived(float[] vectorAction)
{
    //Niet doen
    if (vectorAction[0] == 0)
    {
        AddReward(-0.1f);
    }

    //Achteruit
    if (vectorAction[0] == 1)
    {
        AddReward(-0.001f);
    }

    //Vooruit
    if (vectorAction[0] == 2)
    {
        AddReward(+0.0001f);
    }

    float horizontal = vectorAction[1] == 0 ? 0f : vectorAction[1] * 2 - 3;
    float vertical = vectorAction[0] == 0 ? 0f : vectorAction[0] * 2 - 3;

    Move(horizontal, vertical, vertical, 0f);
}
```

- De verschillende acties waaruit de Agent kan kiezen.

```c#
public override void Heuristic(float[] actionsOut)
{
    actionsOut[0] = 0;
    actionsOut[1] = 0;

    if (Input.GetKey(KeyCode.UpArrow) || Input.GetKey(KeyCode.W))
    {
        actionsOut[0] = 2;
    }

    if (Input.GetKey(KeyCode.DownArrow) || Input.GetKey(KeyCode.S))
    {
        actionsOut[0] = 1;
    }

    if (Input.GetKey(KeyCode.RightArrow) || Input.GetKey(KeyCode.D))
    {
        actionsOut[1] = 2;
    }

    if (Input.GetKey(KeyCode.LeftArrow) || Input.GetKey(KeyCode.A))
    {
        actionsOut[1] = 1;
    }
}
```

- Wanneer er een nieuwe episode begint wordt de Agent terug gezet naar de beginpositie.

```c#
public override void OnEpisodeBegin()
{
    Reset();
}
```

- Beginpositie wordt hier vastgesteld.

```c#
private void Reset()
{
    transform.position = startingPosition;
    transform.rotation = startingRotation;
    rBody.velocity = Vector3.zero;
    OnReset?.Invoke();
}
```

- Op elk circuit zijn 4 vaste checkpoints ingebouwd, wanneer een auto hier voorbij rijd veranderd de status hiervan. Pas als alle 4 de checkpoints zijn gepasseerd kan er over de finish worden gereden en wordt het volgende circuit geladen. In deze versie zijn er ook nog gewone checkpoints die de Agent een beloning geven als die deze checkpoints passeert. In deze versie zijn ook twee soorten finishline. De gewone FinishLine tag staat op het finish object in een echte race. Hierdoor zal het volgende circuit ingeladen worden. Bij het trainen hoort de tag FisnishLineTraining op het object Finishline gezet te worden. Hierdoor zal de episode eindigen en een nieuwe episode starten.

```c#
private void OnTriggerEnter(Collider other)
{
    if (other.gameObject.CompareTag("Checkpoint1"))
    {
        checkpoint1 = true;
        AddReward(1f);
    }

    if (other.gameObject.CompareTag("Checkpoint2"))
    {
        checkpoint2 = true;
        AddReward(1f);
    }

    if (other.gameObject.CompareTag("Checkpoint3"))
    {
        checkpoint3 = true;
        AddReward(1f);
    }

    if (other.gameObject.CompareTag("Checkpoint4"))
    {
        checkpoint4 = true;
        AddReward(1f);
    }
    if (other.gameObject.CompareTag("Checkpoint"))
    {
        AddReward(1f);
    }

    if (other.gameObject.CompareTag("TrainingFinishLine") && checkpoint1 == true && checkpoint2 == true && checkpoint3 == true && checkpoint4 == true)
    {
        AddReward(2f);
        EndEpisode();
    }
    if (other.gameObject.CompareTag("FinishLine") && checkpoint1 == true && checkpoint2 == true && checkpoint3 == true && checkpoint4 == true)
    {
        UnityEngine.SceneManagement.SceneManager.LoadScene(UnityEngine.SceneManagement.SceneManager.GetActiveScene().buildIndex + 1);
    }
}
```

- Bij het in contact komen met een muur zal de Agent een negatieve beloning krijgen.

```c#
    private void OnCollisionEnter(Collision other)
    {
        if (other.gameObject.CompareTag("Wall"))
        {
            AddReward(-2f);
            EndEpisode();
        }

        Physics.IgnoreLayerCollision(1 , 2, true);
    }
}
```

##### Camera Follow

> REFERENTIE Dit script bevat code van de Camera Follow in de 'Unity Standard Assets' asset. Zie bronvermelding.

- Dit script zorgt ervoor dat de camera van de XR Rig de auto van de player blijft volgen.

```c#
public class CameraFollow : MonoBehaviour
{
public Transform target;
public float distance = 3.0f;
public float height = 3.0f;
public float damping = 5.0f;
public bool smoothRotation = true;
public float rotationDamping = 10.0f;

void Update()
{
    Vector3 wantedPosition = target.TransformPoint(0, height, -distance);
    transform.position = Vector3.Lerp(transform.position, wantedPosition, Time.deltaTime * damping);

    if (smoothRotation)
    {
        Quaternion wantedRotation = Quaternion.LookRotation(target.position - transform.position, target.up);
        transform.rotation = Quaternion.Slerp(transform.rotation, wantedRotation, Time.deltaTime * rotationDamping);
    }

    else transform.LookAt(target, target.up);
}
```

##### Timer End

- Dit script is een timer die ervoor zorgt dat het spel reset wanneer het laatste circuit doorlopen is.

```c#
public class TimerEnd : MonoBehaviour
{
    public float countdown = 5;
    void Update()
    {
        countdown = countdown - Time.deltaTime;

        if (countdown < 0)
        {
            SceneManager.LoadScene(0);
        }
    }
}
```

##### Timer Start

- Dit script is een timer die ervoor zorgt dat het spel begint wanneer de simulatie gestart wordt.

```C#
public class TimerStart : MonoBehaviour
{
    public float countdown = 5;
    void Update()
    {
        countdown = countdown - Time.deltaTime;

        if (countdown < 0)
        {
            SceneManager.LoadScene(1);
        }
    }
}
```

#### 2.6 Afwijking one-pager

Er zijn een aantal afwijkingen gemaakt in het project tegenover de one-pager.

| One-pager                                                                                                                                                                                                                                  | Resultaat                                                                                                                                       | Reden                                                                                                                         |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------- |
| De Agent leert om het circuit af te rijden zonder muren of obstakels aan te raken.                                                                                                                                                         | Obstakels zijn niet meer van toepassing.                                                                                                        | Het trainen van de Agent nam veel meer tijd in beslag dan gedacht, hierdoor was er geen tijd meer over voor deze uitbreiding. |
| In het eerste level zal de Agent nog niet heel geleerd zijn in het spel waardoor de gebruiker dus meer kans maakt. In een later level gaan we de Agent een geleerd brein toekennen waardoor er een grotere uitdaging is voor de gebruiker. | We werken met één brein van hetzelfde niveau in plaats van meerdere breinen.                                                                    | Het trainen van de Agent nam veel meer tijd in beslag dan gedacht, hierdoor was er geen tijd meer over voor deze uitbreiding. |
| De gebruiker kan doormiddel van de pijltjestoetsen de auto besturen.                                                                                                                                                                       | Aangezien het de bedoeling is om met de VR-bril te racen is de auto bestuurbaar met de Oculus controllers in plaats van met de pijltjestoetsen. | Het trainen van de Agent nam veel meer tijd in beslag dan gedacht, hierdoor was er geen tijd meer over voor deze uitbreiding. |

### 3. Trainen

Voor het trainen van de ML Agents is er in de custom asset package een scene voorzien. Het is ook mogelijk om de Agent te trainen op een eigen circuit.

Voor het trainen van de Agent is volgende YAML configuratie gebruikt:

```C#
behaviors:
  Racecar:
    trainer_type: ppo
    max_steps: 5e6
    time_horizon: 64
    summary_freq: 10000
    keep_checkpoints: 5
    checkpoint_interval: 50000

    hyperparameters:
      batch_size: 32
      buffer_size: 9600
      learning_rate: 3.0e-4
      learning_rate_schedule: constant
      beta: 5.0e-3
      epsilon: 0.2
      lambd: 0.95
      num_epoch: 3

    network_settings:
      num_layers: 2
      hidden_units: 128
      normalize: false
      vis_encoder_type: simple

    reward_signals:
      extrinsic:
        strength: 1.0
        gamma: 0.99
      curiosity:
        strength: 0.1
        gamma: 0.9999
        encoding_size: 256
        learning_rate: 1e-3

```

Deze configuratie kan u terugvinden in de Learning map van het project. Vervolgens moet u een console naar keuze openen (CMD, Anaconda, Miniconda etc) en de Python omgeving activeren daarna moet u navigeren naar de Learning map binnen het project.

Vervolgens voert u volgend commando uit: mlagents-learn Racecar.yml --run-id racer01

De Racecar.yml verwijst hier naar het configuratiebestand.

racer01 verwijst naar de naam van de training (Deze kan u zelf kiezen).

Daarna moet u in Unity op play drukken en zal de Player beginnen trainen. U kan de resultaten van de training zien door Tensorboard te openen met volgende commando: tensorboard --logdir results

U kan dan navigeren in een internetbrowser naar keuze naar localhost:6006 en daar zal u alle data vinden over de training.

### 4. Resultaten

Er zijn bijna 100! runs gedaan. Tot nu toe geen enkele met optimaal resultaat.
Hieronder zijn 12 runs, de andere runs zijn heel gelijkwaardig met deze runs. Deze runs met kleine aanpassingen van het beloningssysteem maar resulteren bijna altijd in hetzelfde resultaat.

- Run 1
  ![Run1](./images/Run1.png)
- Run 2
  ![Run2](./images/Run2.png)
- Run 3
  ![Run3](./images/Run3.png)
- Run 4
  ![Run4](./images/Run4.png)
- Run 5
  ![Run5](./images/Run5.png)
- Run 6
  ![Run6](./images/Run6.png)
- Run 7
  ![Run7](./images/Run7.png)
- Run 8
  ![Run8](./images/Run8.png)
- Run 9
  ![Run9](./images/Run9.png)
- Run 10
  ![Run10](./images/Run10.png)
- Run 11
  ![Run11](./images/Run11.png)
- Run 12
  ![Run12](./images/Run12.png)

Wat opvalt is dat De Agent ergens een hoogtepunt bereikt en daarna altijd slechter begint te rijden. Af en toe wordt hij daarna weer iets beter maar uiteindelijk is het einde altijd hetzelfde, de agent bereikt op het einde bijna altijd een minimum.

### 5. Conclusie

Wij hebben geprobeerd een Agent te trainen die op de snelst mogelijke manier verschillende circuits kan afleggen doormiddel van één brein.

Wij hebben geprobeerd een Agent te trainen die op de snelst mogelijke manier verschillende circuits kan afleggen doormiddel van één brein.

Zoals in de resultaten vermeld en zichtbaar is, haalt de Agent een hoogtepunt en daarna gaat het bergaf. We denken dat dit komt door het feit dat eerst de Agent de bochten moet leren doorrijden. Eens dat de Agent het volledige circuit heeft doorgereden zit hij dus op bijna het maximaal aantal reward. De Agent krijgt meer punten hoe sneller hij het circuit aflegt en dit begint de Agent door te krijgen, hierdoor gaat de agent sneller en sneller rijden en uiteindelijk rijdt hij te snel om de bocht te kunnen nemen en slipt hij uit de bocht. In geen enkele van de runs dat we gedaan hebben heeft de Agent geleerd dat hij moet remmen om de bocht te nemen.

Wij zijn er dus niet in geslaagd een geoptimaliseerde Agent te ontwikkelen, wij vinden dat we niet tot een geoptimaliseerd beloningssyteem zijn geraakt en ongetwijfeld zullen er nog andere factoren zijn die ervoor zorgen dat onze Agent niet optimaal is getraind.

### 6. Bronvermelding

- Unity Technologies (Apr 8, 2018). Unity Standard Assets. <https://assetstore.unity.com/packages/essentials/asset-packs/standard-assets-for-unity-2018-4-32351>
- Unity Technologies (Jan 5, 2021). Unity XR Input. <https://docs.unity3d.com/Manual/xr_input.html>
