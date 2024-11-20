using UnityEngine;

public static class RPSRule
{
    public static (RPSLogic winner, RPSLogic loser, bool isTie) JudgeWinner(RPSLogic A, RPSLogic B)
    {
        if (A.Type == B.Type)
        {
            return (null, null, true);
        }

        if(A.Type == RPSType.R)
        {
            if(B.Type == RPSType.P)
            {
                return (B, A, false);
            }
            
            if(B.Type == RPSType.S)
            {
                return (A, B, false);
            }
        }

        if (A.Type == RPSType.P)
        {
            if (B.Type == RPSType.R)
            {
                return (A, B, false);
            }

            if (B.Type == RPSType.S)
            {
                return (B, A, false);
            }
        }

        if(A.Type == RPSType.S)
        {
            if (B.Type == RPSType.P)
            {
                return (A, B, false);
            }

            if (B.Type == RPSType.R)
            {
                return (B, A, false);
            }
        }

        return (null, null, false);
    }
}
