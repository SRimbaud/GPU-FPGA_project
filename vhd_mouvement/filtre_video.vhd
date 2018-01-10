library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity filtre_video is
	generic (
		size	: integer := 9 	-- taille de la sous-fenetre = 2**size pixels
		);
    	port (
		--horloge et reset
		CLK			: in std_logic; -- clock � 54 MHz
		RESET 		: in std_logic; -- reset � 0 			
		-- flux video � 27 MHz
		-- synchro
		VGA_X :	in std_logic_vector(10 downto 0); -- compteur pixels
		VGA_Y :	in std_logic_vector(10 downto 0); -- compteur lignes
		-- entr�e
		iY : 	in std_logic_vector(7 downto 0); -- flux video entrant : luminance
		iCb : 	in std_logic_vector(7 downto 0); -- flux video entrant : chrominance bleu
		iCr : 	in std_logic_vector(7 downto 0); -- flux video entrant : chrominance rouge
		-- sortie
		oY	: 	out std_logic_vector(7 downto 0); -- flux video sortant : luminance
		oCb	: 	out std_logic_vector(7 downto 0); -- flux video sortant : chrominance bleu
		oCr	: 	out std_logic_vector(7 downto 0); -- flux video sortant : chrominance rouge
		--switch D2E
		switch			: in std_logic_vector(17 downto 0);		-- � connecter � DPDT_SW;
		-- SRAM interfaces
		address_SRAM 	: out std_logic_vector(17 downto 0); 	-- � connecter � SRAM_ADDR
		data_SRAM		: inout std_logic_vector(15 downto 0);  -- � connecter � SRAM_DQ
		write_enable 	: out std_logic; 						-- � connecter � SRAM_WE_N
		read_enable 	: out std_logic; 						-- � connecter � SRAM_OE_N
		chip_enable 	: out std_logic;						-- � connecter � SRAM_CE_N 
		high_mask 		: out std_logic ; 						-- � connecter � SRAM_UB_N
		low_mask 		: out std_logic 			
		);			
end entity filtre_video;


architecture A of filtre_video is

component memoire_ligne 
		generic (
		address_size : integer;
		word_size : integer
		);
	    port (
		CLK			: in std_logic;
		RESET		: in std_logic;		
		address 	: in std_logic_vector(address_size-1 downto 0);
		data_in		: in std_logic_vector(word_size-1 downto 0);
		data_out	: out std_logic_vector(word_size-1 downto 0);
		read_write	: in std_logic
		);
end component ;


component module_fenetrage -- Sort un signal in_active_area
	generic (
			size			: integer := 8
			);
	port (
			VGA_X 			:	in std_logic_vector(10 downto 0);
			VGA_Y 			:	in std_logic_vector(10 downto 0);			
			iY 				: 	in std_logic_vector(7 downto 0);
			oY				: 	out std_logic_vector(7 downto 0);
			in_active_area 	:	out std_logic; -- Passe � 1 pendant 521 pixs et revient � 0(c'est un esp�ce d'enable du process).
			X_cpt			:	out std_logic_vector(10 downto 0);
			Y_cpt			:	out std_logic_vector(10 downto 0)
		);
end component;


-- component module_SRAM -- On va pas se servir de ce module du coup on le commente. 
--     port (
-- 			CLK				: 	in std_logic;
-- 			RESET 			: 	in std_logic;			 
-- 			--module top interface
-- 			in_active_area	:	in std_logic;
-- 			X_cpt			: 	in std_logic_vector(10 downto 0);
-- 			Y_cpt			: 	in std_logic_vector(10 downto 0);
-- 			data_in			:	in std_logic_vector(7 downto 0);
-- 			data_out		: 	out std_logic_vector(7 downto 0);					
-- 			-- SRAM interfaces
-- 			address_SRAM 	: out std_logic_vector(17 downto 0); 	-- � connecter � SRAM_ADDR
-- 			data_SRAM		: inout std_logic_vector(15 downto 0);  -- � connecter � SRAM_DQ
-- 			write_enable 	: out std_logic; 						-- � connecter � SRAM_WE_N
-- 			read_enable 	: out std_logic; 						-- � connecter � SRAM_OE_N
-- 			chip_enable 	: out std_logic;						-- � connecter � SRAM_CE_N 
-- 			high_mask 		: out std_logic ; 						-- � connecter � SRAM_UB_N
-- 			low_mask 		: out std_logic  						-- � connecter � SRAM_LB_N
-- 			);			
-- end component;


component module_diff -- C'est celui qui fait notre process.
  port (
	in_active_area 		: 	in 	std_logic ;
	iY1					:	in	std_logic_vector(7 downto 0) ;
	iY2					:	in	std_logic_vector(7 downto 0) ;
	oY					: 	out	std_logic_vector(7 downto 0) ;
	threshold			: 	in 	std_logic_vector(7 downto 0) 
  ) ;
end component ; -- module_diff

--signaux flux video
signal sig_Y1			: std_logic_vector(7 downto 0) ;
signal sig_Y2			: std_logic_vector(7 downto 0) ;
signal sig_Y3			: std_logic_vector(7 downto 0) ;
signal sig_concat : std_logic_vector(17 downto 0);
signal mem2_vers_mem3 : std_logic_vector(17 downto 0);
signal sortie_finale : std_logic_vector(17 downto 0);

--signaux de synchro module fentrage
signal Y_cpt			: std_logic_vector(10 downto 0);
signal X_cpt 			: std_logic_vector(10 downto 0);
signal in_active_area 		: std_logic;

--signaux debug
signal threshold		: std_logic_vector(7 downto 0) ;

--signuax interm�diares
signal address 			:std_logic_vector( size -1 downto 0) := (others => '0');
signal read_write		:std_logic;

begin
	u_1: module_fenetrage 
	generic map(
			size => size
			)
	port map(
			VGA_X => VGA_X,
			VGA_Y => VGA_Y,			
			iY	=> iY,
			oY	=> sig_Y1,
			in_active_area => in_active_area,
			X_cpt => X_cpt,
			Y_cpt => Y_cpt
			);
			
	mem1 :memoire_ligne 
		generic map (
			address_size => size,
			word_size => 8 
			)
		port map (
			CLK		=> CLK,
			RESET		=> RESET, 
			address 	=> address,		
			data_in		=> iY,
			data_out	=> sig_Y2,
			read_write	=> read_write
			);

	mem2 :memoire_ligne 
		generic map (
			address_size => size,
			word_size => 18 
			)
		port map (
			CLK		=> CLK,
			RESET		=> RESET, 
			address 	=> address,
			data_in		=> sig_concat,
			data_out	=> mem2_vers_mem3,
			read_write	=> read_write
			);
	mem3 :memoire_ligne 
		generic map (
			address_size => size,
			word_size => 18 
			)
		port map (
			CLK		=> CLK,
			RESET		=> RESET, 
			address 	=> address,
			data_in		=> mem2_vers_mem3,
			data_out	=> sortie_finale,
			read_write	=> read_write
			);


	gradient : process (clk,in_active_area)

  variable reg2: std_logic_vector(7 downto 0) := (others => '0');
	variable data_in: std_logic_vector(7 downto 0):= (others => '0');
	variable Gx: signed(8 downto 0) := (others => '0');
	variable Gy: signed(8 downto 0) := (others => '0');
	variable G: signed(17 downto 0) := (others => '0');
	variable orientation: std_logic_vector(1 downto 0) := (others => '0');
	
	
	variable regA: std_logic_vector(15 downto 0) := (others => '0');
	variable regB: std_logic_vector(17 downto 0) := (others => '0');
	-- regB plus grand besoin de son orientation (pixel central)
	variable regC: std_logic_vector(15 downto 0) := (others => '0');
	variable regD: std_logic_vector(15 downto 0) := (others => '0');
	variable regE: std_logic_vector(15 downto 0) := (others => '0');
	variable regF: std_logic_vector(15 downto 0) := (others => '0');
	begin
	
	read_write <= '1'; -- amodifier pour faire le calcul tous les deux fronts d'horloge
	-- Quand il est a 0 on est en trian de modi fla m�moire, on travaille en lisant
	-- la m�moire que quand il est a Un sinon c'est d�bile as FUCK
	--if in_active_area != 1 alors remettre registre a 0
	
		
		if (clk='1' and clk'event and in_active_area = '1') then
		  --read_write <= not read_write
			if read_write = '1' then
			  --reg1 := data_in;
			  
				Gy := -signed('0'&iY) + signed('0'&sig_Y2) -signed('0'&data_in) + signed('0'&reg2); 
				Gx := -signed('0'&iY) - signed('0'&sig_Y2) +signed('0'&data_in) + signed('0'&reg2);
				G := Gx*Gx + Gy*Gy;
				address <= std_logic_vector(unsigned(address) + 1) ;

				if Gx*Gx > 4 * Gy*Gy then
					orientation := "10";
				elsif Gy*Gy > 4* Gx*Gx then
					orientation := "00";
				elsif Gx*Gy > 0 then
					orientation := "01";
				else
					orientation := "11";

				end if;
				
				G := G(17 downto 2)&signed(orientation);
				sig_concat <= std_logic_vector(G) ;
				
				-- Comparaison des pixels selon une direction.
				-- On travaille avec l'orientation du pixel 5 donc celui dans registre B.
				
				if regB(1 downto 0) = "00" and regB(17 downto 2)>= regA and regB(17 downto 2) >= regC then --verticale 6,4
				 
				    sig_Y3 <= (others => '1' );--std_logic_vector(G(17 downto 10)) ;
				    
				    
				elsif regB(1 downto 0) = "01" then -- 1 et 9  deuxi�me bissectrice
				  if regB(17 downto 2) >= regD and regB(17 downto 2) >= sig_concat(17 downto 2) then
				    sig_Y3 <= (others => '1' );--std_logic_vector(G(17 downto 2))  ;
				    end if ;
				    
				elsif regB(1 downto 0) = "10" then -- horizontale 8,2
				  if regB(17 downto 2)>= regE and regB(17 downto 2)>= mem2_vers_mem3(17 downto 2) then
				    sig_Y3 <= (others => '1' );--std_logic_vector(G(17 downto 2))  ;
				    end if ;
				    
				elsif regB(1 downto 0) = "11" then-- permi�re bissectrice 7,3
				  if regB(17 downto 2)>= regF and regB(17 downto 2)>= sortie_finale(17 downto 2) then
				    sig_Y3 <= (others => '1' ); --std_logic_vector(G(17 downto 2))  ;
				    end if;
				    
				else 
				  sig_Y3 <=(others => '0')  ;
				  
			  end if;
				       
				  
				  
				

			end if;
			reg2 := sig_Y2;
			data_in := iY;
			--Mise � jour des regitres
			
			-- On mets � jour les registres qui stockent les vals + anciennes
			-- Pour �viter des les perdres
			regF := regC;
			regE := regB(17 downto 2);
			regD := regA;
			
			--Registre les plus r�cents qui stockent signaux qu'on vient de d'�mettre.
			regC := sig_concat(17 downto 2);
			regA := sortie_finale(17 downto 2);
			regB := mem2_vers_mem3;
			
			
			
			
		
		else 
		  oY <= (others => '0');
		end if;
	end process;

	--concurent
	threshold <= switch(17 downto 10);
	oCb <= X"80";		
	oCr <= X"80";

	--process
	process_affichage : process( switch, iY, sig_Y1, sig_Y2, sig_Y3)
	begin
		case( switch(4 downto 0) ) is
			when "00000" => oY <= iY; -- avant fenetrage		
			when "00001" => oY <= sig_Y1; -- apr�s fenetrage				
			when "00011" => oY <= sig_Y2; -- apr�s memoire
			when others  => oY <= sig_Y3;  -- apr�s diff
		end case ;
	end process ; -- process_affichage

end architecture A;	
