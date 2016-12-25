namespace HeliSharp
{
    public interface IHelicopterControls
    {
        double Collective { get; set; }
        double LongCyclic { get; set; }
        double LatCyclic { get; set; }
        double Pedal { get; set; }
    }
}