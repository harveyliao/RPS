using UnityEngine;

public enum RPSType
{
    R, P, S
}

// This script is used to control the logic of the Rock-Paper-Scissors game
public class RPSLogic : MonoBehaviour
{
    private RPSType type;
    public RPSType Type {  get { return type; } }

    private RPSView view;
    private Rigidbody2D rb;
    private bool isDragging = false;
    private Vector2 offset;

    private void Awake()
    {
        view = GetComponentInChildren<RPSView>();
        rb = GetComponent<Rigidbody2D>();
        
        // Configure Rigidbody2D for mouse movement
        rb.gravityScale = 0;
        rb.linearDamping = 5;          // Add some drag to make movement smoother
        rb.collisionDetectionMode = CollisionDetectionMode2D.Continuous;  // Better collision detection
    }

    public void Start()
    {
        // assign self with random type
        type = (RPSType) Random.Range(0, 3);

        // initial update visual
        view.SetEmoji(type);
    }

    public void LoseTo(RPSLogic winner)
    {
        type = winner.type;
        view.SetEmoji(winner.type);
    }

    private void OnCollisionEnter2D(Collision2D collision)
    {
        Debug.Log($"Collision detected between {gameObject.name} and {collision.gameObject.name}");

        RPSLogic otherLogic = collision.gameObject.GetComponent<RPSLogic>();
        if (otherLogic != null)
        {
            (RPSLogic winner, RPSLogic loser, bool isTie) = RPSRule.JudgeWinner(this, otherLogic);

            if (!isTie)
            {
                loser.LoseTo(winner);
            }
        }
    }

    private void OnMouseDown()
    {
        isDragging = true;
        offset = rb.position - GetMouseWorldPosition();
    }

    private void OnMouseUp()
    {
        isDragging = false;
    }

    private Vector2 GetMouseWorldPosition()
    {
        Vector3 mousePos = Camera.main.ScreenToWorldPoint(Input.mousePosition);
        return new Vector2(mousePos.x, mousePos.y);
    }

    private void Update()
    {
        if (isDragging)
        {
            Vector2 targetPos = GetMouseWorldPosition() + offset;
            rb.MovePosition(targetPos);
        }
    }
}
