    addi $t7, $zero, $zero # nop
    addi	$t0, $zero, 0x34			# $t0 = $t1 + 0
    addi $t7, $zero, $zero # nop
    j 0x1c
    addi $t7, $zero, $zero # nop
    addi $t7, $zero, $zero # nop
    addi	$t1, $zero, 1			# $t0 = $t1 + 0
    addi $t7, $zero, $zero # nop
    addi $t7, $zero, $zero # nop
    jr		$t0					# jump to $ra
    addi $t7, $zero, $zero # nop
    addi $t7, $zero, $zero # nop
    addi	$t1, $zero, 2			# $t0 = $t1 + 0
    addi $t7, $zero, $zero # nop
    jal 0x44
    addi $t7, $zero, $zero # nop
    addi $t7, $zero, $zero # nop
    addi	$t1, $zero, 2			# $t0 = $t1 + 0
    addi $t7, $zero, $zero # nop
    addi	$t0, $zero, 0x1			# $t0 = $t1 + 0
    addi	$t1, $zero, 0x2			# $t0 = $t1 + 0
    add  $t2, $t1, $t0
    sub  $t2, $t1, $t0
    and  $t2, $t1, $t0
    or  $t2, $t1, $t0
    slt  $t2, $t1, $t0
    slt  $t2, $t0, $t1
    sw $t1, 0x20($zero)
    lw $t2, 0x20($zero)
    beq $t0, $t1, branch_eq
    beq $t0, $t0, branch_eq
    addi $t7, $zero, $zero # nop
branch_eq:
    multu $t0, $t1
    mflo $t2
    mfhi $t2
    sll $t2, $t1, 0x1
    srl $t2, $t2, 0x2
end:


# Instructions to test
# *add, sub, and, or, slt, lw, sw, beq, *j
# multu, mfhi, mflo, *jr, *jal, sll, srl  
